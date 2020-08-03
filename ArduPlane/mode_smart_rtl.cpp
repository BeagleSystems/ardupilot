#include "mode.h"
#include "Plane.h"

/*
// Smart RTL is designed for the scenario which an unexpected failsafe happends during an AUTO mission,
// Normal RTL will directly fly to nearest RALLY point or HOME position.
// Problems are:
// 1. if there is a GCS loss caused failsafe, we dont know which RALLY point the drone will go to, only guess.
// 2. if there is obstacle high enough on the way between current location and RALLY/HOME position, the drone will crash.
// 3. if there is restricted area on the way between, we will break the rule of non-fly zone.
// The idea of smart RTL is if drone cleared any of way points from AUTO mission, it just need to fly back exactly as how it went there.
*/

bool ModeSmartRTL::_enter()
{
    if(plane.mission.state() != AP_Mission::MISSION_STOPPED)
    {
        // if no waypoints in history or not auto mission, go normal rtl
        plane.set_mode(plane.mode_rtl, ModeReason::SMART_RTL_SWITCHING_TO_RTL);
    }
    

    // else set mission state to rewind

    return true;
}

void ModeSmartRTL::update()
{
    plane.calc_nav_roll();
    plane.calc_nav_pitch();
    plane.calc_throttle();

    if (plane.g2.rtl_climb_min > 0) {
        /*
          when RTL first starts limit bank angle to LEVEL_ROLL_LIMIT
          until we have climbed by RTL_CLIMB_MIN meters
         */
        if (!plane.rtl.done_climb && (plane.current_loc.alt - plane.prev_WP_loc.alt)*0.01 > plane.g2.rtl_climb_min) {
            plane.rtl.done_climb = true;
        }
        if (!plane.rtl.done_climb) {
            plane.roll_limit_cd = MIN(plane.roll_limit_cd, plane.g.level_roll_limit*100);
            plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -plane.roll_limit_cd, plane.roll_limit_cd);
        }
    }
}


#if false

bool ModeAuto::_enter()
{
    plane.throttle_allows_nudging = true;
    plane.auto_throttle_mode = true;
    plane.auto_navigation_mode = true;
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if SOARING_ENABLED == ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND &&
            !plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))
        {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
    } else if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        } else {
            plane.calc_throttle();
        }
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

#endif
