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
    // check mission state to find out if smart RTL is needed
    // if current nav cmd index in list is not 0
    uint16_t cur_idx = plane.mission.get_current_nav_index();
    gcs().send_text(MAV_SEVERITY_INFO, "Enter SmartRTL mode with wp idx=%u", static_cast<unsigned>(cur_idx));

    if(plane.mission.get_current_nav_index() != 0)
    {
        plane.throttle_allows_nudging = true;
        plane.auto_throttle_mode = true;
        plane.auto_navigation_mode = true;

        plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
        // start or resume the mission, based on MIS_AUTORESET
        //uint16_t cur_idx = plane.mission.get_current_nav_index();
        plane.mission.set_force_resume(true);
        //plane.mission.start_or_resume();

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
        // ready to do smart RTL
        // plane.throttle_allows_nudging = true;
        // plane.auto_throttle_mode = true;
        // plane.auto_navigation_mode = true;
        // plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;

        // //set mission state to flyback
        // uint16_t cur_idx = plane.mission.get_current_nav_index();
        // gcs().send_text(MAV_SEVERITY_INFO, "Enter SmartRTL mode with wp idx=%u", static_cast<unsigned>(cur_idx));
        // //waypoints rewind will be handled in Plane::update_navigation() in task SCHED_TASK(navigate,10,150)
        // // plane.mission.set_current_cmd(cur_idx);
        // plane.mission.start_fly_back(cur_idx);
    }
    else // go back to normal RTL
    {
        plane.set_mode(plane.mode_rtl, ModeReason::SMART_RTL_SWITCHING_TO_RTL);
    }

    return true;
}

void ModeSmartRTL::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeSmartRTL::update()
{

    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        AP_Mission::mission_state temp_stat = plane.mission.state();
        // gcs().send_text(MAV_SEVERITY_INFO, "mission state + %u", temp_stat);
        plane.mission.resume_flyback();
        
        //plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        //gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in srtl without a running mission");
        return;
    }

    // AP_Mission::mission_state temp_stat = plane.mission.state();
    // gcs().send_text(MAV_SEVERITY_INFO, "mission state restarted + %u", temp_stat);
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
