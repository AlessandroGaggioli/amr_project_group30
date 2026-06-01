"""
Common state implementations shared by Task 2 and Task 3 state machines.

This mixin expects the consumer class to expose the following
attributes: `node`, `arm`, `nav`, `amcl`, `aruco`, `sampler`,
`state`, `finished`, `search_phase`, `_send_time` and any helper
flags the original implementations used.
"""
import math
import time

from tiago_project_group30.task2_constants import (
    SEARCH_PAN_DWELL,
    SEARCH_PAN_POSITIONS,
)


class CommonStates:
    # ------------------------------------------------------------------
    # State 0: tuck arm to HOME
    # ------------------------------------------------------------------
    def state_0_arm_tuck(self):
        self.node.get_logger().info("State 0: tuck arm to HOME (MoveIt)")
        self.arm.move_to_home(tilt_head=False)
        self._send_time = time.time()
        self.state = 1

    # ------------------------------------------------------------------
    # State 1: wait for arm motion to finish
    # ------------------------------------------------------------------
    def state_1_wait_arm(self, planning_timeout):
        if self.arm.motion_done:
            self.node.get_logger().info(
                "Arm at HOME, starting AMCL global localization"
            )
            self.state = 2
        elif (
            not self.arm.motion_started
            and (time.time() - self._send_time) > planning_timeout
        ):
            self.node.get_logger().warn("Arm planning timed out, retrying")
            self.state = 0

    # ------------------------------------------------------------------
    # State 2: AMCL global localization
    # ------------------------------------------------------------------
    def state_2_amcl_localization(self):
        amcl = self.amcl
        if amcl.loc_substate == 0:
            self.node.get_logger().info(
                "State 2a: requesting AMCL global localization"
            )
            if amcl.request_global_localization():
                # Give AMCL a moment to redistribute particles before any
                # further action.
                time.sleep(2.0)
                amcl.loc_substate = 1
            else:
                time.sleep(2.0)  # service unavailable -> retry
        elif amcl.loc_substate == 1:
            self.node.get_logger().info(
                "State 2b: clearing local costmap before spin"
            )
            amcl.clear_local_costmap()
            time.sleep(0.5)  # let the costmap process the clear
            amcl.loc_substate = 2
        elif amcl.loc_substate == 2:
            amcl.localization_spin_count += 1
            self.node.get_logger().info(
                f"State 2c: spin attempt "
                f"#{amcl.localization_spin_count} (target_yaw=2*pi)"
            )
            amcl.send_spin(2.0 * math.pi)
            self._send_time = time.time()
            amcl.loc_substate = 3
        elif amcl.loc_substate == 3:
            # Convergence wins over spin completion: as soon as AMCL is
            # confident we can move on, even mid-spin.
            if amcl.converged:
                self.node.get_logger().info(
                    "State 2d: AMCL aligned -- proceeding to search"
                )
                if amcl.spin_active and amcl.spin_goal_handle is not None:
                    amcl.cancel_spin()
                # Note: clear_global_costmap() is fired automatically
                # inside update_amcl_flags() on the convergence transition,
                # so it's already been called by this point.
                self.state = 3
                return
            if amcl.spin_done:
                # Spin finished but AMCL not yet converged.
                if amcl.localization_spin_count >= 3:
                    self.node.get_logger().warn(
                        f"AMCL not converged after "
                        f"{amcl.localization_spin_count} spins, "
                        "proceeding anyway"
                    )
                    self.state = 3
                    return
                self.node.get_logger().info(
                    "Spin done, AMCL not converged yet -- "
                    "clearing costmap and re-spinning"
                )
                amcl.spin_done = False
                amcl.loc_substate = 1  # back to clear costmap

    # ------------------------------------------------------------------
    # State 3: random search w/ visited memory + per-waypoint head pan
    # ------------------------------------------------------------------
    def state_3_random_search(self, search_nav_timeout):
        # Returns True if a `continue` should be emitted (timeout watcher
        # fired or waiting for costmap), False otherwise.
        #
        # Per-waypoint sweep uses a HEAD PAN sequence (see task2_constants
        # SEARCH_PAN_POSITIONS / SEARCH_PAN_DWELL) instead of a Nav2 /spin.
        # The /spin behavior frequently aborts on DWB collision /
        # oscillation critics, every abort sloshes AMCL, and a successful
        # spin takes 6-15 s. The pan is fire-and-forget on the head
        # controller, doesn't touch the base, and dwells just long enough
        # for the ArUco PnP to converge.
        nav = self.nav
        aruco = self.aruco
        arm = self.arm

        # Exit condition: both approach poses latched.
        if (
            aruco.pick_approach_pose is not None
            and aruco.place_approach_pose is not None
        ):
            if nav.goal_active:
                self.node.get_logger().info(
                    "Both markers seen mid-search, cancelling nav"
                )
                nav.cancel()
            # Clear ALL goal state so a stale callback from the
            # cancelled in-flight goal doesn't bleed into State 4.
            nav.goal_active = False
            nav.goal_done = False
            nav.goal_succeeded = False
            # Re-centre the head so subsequent states (and Task 3) start
            # from a known pan=0 pose.
            arm.tilt_head(-0.5, 0.0)
            self.search_phase = "sample"
            self._pan_idx = 0
            # Freeze the approach poses now so Nav2's target doesn't
            # keep drifting during the final approach.
            aruco.freeze()
            self.node.get_logger().info(
                f"Freezing PICK approach at "
                f"({aruco.pick_approach_pose.pose.position.x:.2f}, "
                f"{aruco.pick_approach_pose.pose.position.y:.2f}) and "
                f"PLACE approach at "
                f"({aruco.place_approach_pose.pose.position.x:.2f}, "
                f"{aruco.place_approach_pose.pose.position.y:.2f})"
            )
            self.state = 4
            return True

        # Lazy-init the pan cursor on first entry.
        if not hasattr(self, "_pan_idx"):
            self._pan_idx = 0

        # 1. Handle nav goal completion (only relevant in 'nav')
        if self.search_phase == "nav" and nav.goal_done:
            nav_ok = nav.goal_succeeded
            nav.goal_done = False
            if nav_ok:
                # Arrived at waypoint -> start head pan sweep.
                self._pan_idx = 0
                pan = SEARCH_PAN_POSITIONS[self._pan_idx]
                self.node.get_logger().info(
                    f"Waypoint reached, starting head pan sweep "
                    f"[{self._pan_idx + 1}/{len(SEARCH_PAN_POSITIONS)}] "
                    f"pan={pan:.2f} rad"
                )
                arm.tilt_head(-0.5, pan)
                self.search_phase = "pan"
                self._send_time = time.time()
            else:
                # Nav failed -> abandon this waypoint
                self.node.get_logger().warn(
                    "Waypoint nav failed -> sampling next"
                )
                self.search_phase = "sample"

        # 2. Advance pan sweep when dwell time elapses
        if self.search_phase == "pan":
            if (time.time() - self._send_time) >= SEARCH_PAN_DWELL:
                self._pan_idx += 1
                if self._pan_idx >= len(SEARCH_PAN_POSITIONS):
                    # Sweep complete -> sample next waypoint
                    self.search_phase = "sample"
                else:
                    pan = SEARCH_PAN_POSITIONS[self._pan_idx]
                    self.node.get_logger().info(
                        f"Head pan "
                        f"[{self._pan_idx + 1}/{len(SEARCH_PAN_POSITIONS)}] "
                        f"pan={pan:.2f} rad"
                    )
                    arm.tilt_head(-0.5, pan)
                    self._send_time = time.time()

        # 3. Watch nav timeout
        if self.search_phase == "nav" and nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > search_nav_timeout
            ):
                self.node.get_logger().warn(
                    "Nav goal timed out, cancelling -> sample next"
                )
                nav.cancel()
                nav.timeout_fired = True
            return True

        # 4. Idle ('sample') -> draw a new waypoint and send nav
        if self.search_phase == "sample":
            if self.sampler.costmap_msg is None:
                self.node.get_logger().info(
                    "Waiting for /global_costmap/costmap...",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.5)
                return True
            xy = self.sampler.sample_random_xy()
            if xy is None:
                # sample_random_xy returns None for either
                # (a) the visited set covers all reachable cells
                #     -> real saturation, clearing helps;
                # (b) costmap / TF lookup was transiently unavailable
                #     -> clearing visited does nothing useful.
                self.node.get_logger().warn(
                    "Sample returned None, clearing visited "
                    "list and retrying",
                    throttle_duration_sec=2.0,
                )
                self.sampler.visited_waypoints = []
                time.sleep(0.5)
                return True
            x, y , yaw = xy
            self.sampler.visited_waypoints.append((x, y))
            self.node.get_logger().info(
                f"New random waypoint ({x:.2f}, {y:.2f}), {yaw:.2f};"
                f"visited={len(self.sampler.visited_waypoints)}"
            )
            nav.send_goal(x, y, yaw)
            self.search_phase = "nav"
            self._send_time = time.time()
        return False
    
    # # ------------------------------------------------------------------
    # # State 3: MANUAL search via RViz + autonomous head pan sweep
    # # ------------------------------------------------------------------
    # def state_3_random_search(self, search_nav_timeout):
    #     # Il parametro search_nav_timeout viene mantenuto nella firma 
    #     # per non dover modificare la chiamata nel ciclo run(), ma viene ignorato.
    #     aruco = self.aruco
    #     arm = self.arm

    #     # 1. CONDIZIONE DI USCITA: Entrambi i marker sono stati agganciati.
    #     if (
    #         aruco.pick_approach_pose is not None
    #         and aruco.place_approach_pose is not None
    #     ):
    #         self.node.get_logger().info(
    #             "Entrambi i marker visti! Prendo il controllo e interrompo la navigazione RViz."
    #         )
            
    #         # Ricentra la testa in modo che gli stati successivi partano puliti
    #         arm.tilt_head(-0.5, 0.0)
            
    #         # Congela le pose per impedire al rumore di spostare l'approccio
    #         aruco.freeze()
    #         self.node.get_logger().info(
    #             f"Freezing PICK approach at "
    #             f"({aruco.pick_approach_pose.pose.position.x:.2f}, "
    #             f"{aruco.pick_approach_pose.pose.position.y:.2f}) and "
    #             f"PLACE approach at "
    #             f"({aruco.place_approach_pose.pose.position.x:.2f}, "
    #             f"{aruco.place_approach_pose.pose.position.y:.2f})"
    #         )
    #         # Passando allo State 4, il nodo invierà un NUOVO Nav2Goal, 
    #         # che annullerà automaticamente e immediatamente l'eventuale 
    #         # obiettivo RViz in esecuzione.
    #         self.state = 4
    #         return True

    #     # 2. INIZIALIZZAZIONE DELLA RICERCA MANUALE
    #     # Eseguito solo la prima volta che entriamo nello State 3
    #     if getattr(self, "_manual_search_init", None) is None:
    #         self.node.get_logger().info("==================================================")
    #         self.node.get_logger().info("RICERCA MANUALE ATTIVA: Usa 'Nav2 Goal' su RViz!")
    #         self.node.get_logger().info("==================================================")
    #         self._manual_search_init = True
    #         self._pan_idx = 0
    #         self._send_time = time.time()
    #         arm.tilt_head(-0.5, SEARCH_PAN_POSITIONS[self._pan_idx])

    #     # 3. PANNING CONTINUO DELLA TESTA
    #     # Continua a muovere la telecamera a destra e sinistra per cercare i marker
    #     # mentre TU guidi fisicamente il robot dal computer tramite RViz.
    #     if (time.time() - self._send_time) >= SEARCH_PAN_DWELL:
    #         self._pan_idx = (self._pan_idx + 1) % len(SEARCH_PAN_POSITIONS)
    #         pan = SEARCH_PAN_POSITIONS[self._pan_idx]
    #         arm.tilt_head(-0.5, pan)
    #         self._send_time = time.time()
            
    #         # Stampa un log a schermo ogni 5 secondi per ricordarti cosa sta facendo
    #         self.node.get_logger().info(
    #             "In attesa dei marker... Mandami in giro usando Nav2Goal su RViz.",
    #             throttle_duration_sec=5.0
    #         )

    #     return False

    # ------------------------------------------------------------------
    # State 4: navigate to PICK approach
    # ------------------------------------------------------------------
    def state_4_pick(self, approach_nav_timeout):
        nav = self.nav
        # 1. Handle goal completion FIRST.
        if nav.goal_done:
            nav.goal_done = False  # consume
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "PICK approach reached, advancing to PLACE"
                )
                self.state = 5
                return True
            else:
                self.node.get_logger().warn(
                    "PICK nav did not succeed, retrying"
                )
                # fall through -> the (not active) branch below will
                # re-send the goal.

        # 2. In flight -> only watch for timeout.
        if nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > approach_nav_timeout
            ):
                self.node.get_logger().warn(
                    "PICK nav timed out, cancelling for retry"
                )
                nav.cancel()
                nav.timeout_fired = True
            return True

        # 3. Not active -> send (initial or retry).
        x, y, yaw = self.nav.approach_pose_to_xy_yaw(
            self.aruco.pick_approach_pose
        )
        self.node.get_logger().info("State 4: nav to PICK approach")
        nav.send_goal(x, y, yaw)
        self._send_time = time.time()
        return False

    # ------------------------------------------------------------------
    # State 5: navigate to PLACE approach
    # ------------------------------------------------------------------
    def state_5_place(self, approach_nav_timeout):
        nav = self.nav
        # 1. Handle goal completion FIRST.
        if nav.goal_done:
            nav.goal_done = False  # consume
            if nav.goal_succeeded:
                self.node.get_logger().info(
                    "PLACE approach reached, task done"
                )
                self.state = 6
                return True
            else:
                self.node.get_logger().warn(
                    "PLACE nav did not succeed, retrying"
                )

        # 2. In flight -> only watch for timeout.
        if nav.goal_active:
            if (
                not nav.timeout_fired
                and (time.time() - self._send_time) > approach_nav_timeout
            ):
                self.node.get_logger().warn(
                    "PLACE nav timed out, cancelling for retry"
                )
                nav.cancel()
                nav.timeout_fired = True
            return True

        # 3. Not active -> send (initial or retry).
        x, y, yaw = self.nav.approach_pose_to_xy_yaw(
            self.aruco.place_approach_pose
        )
        self.node.get_logger().info("State 5: nav to PLACE approach")
        nav.send_goal(x, y, yaw)
        self._send_time = time.time()
        return False

    # ------------------------------------------------------------------
    # State 6: done
    # ------------------------------------------------------------------
    def state_6_done(self):
        self.node.get_logger().info("State 6: Task 2 completed")
        self.finished = True
