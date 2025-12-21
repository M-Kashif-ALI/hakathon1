---
sidebar_position: 600
title: Error Handling and Fallback Mechanisms
---

# Error Handling and Fallback Mechanisms

This section covers comprehensive error handling and fallback mechanisms for Vision-Language-Action (VLA) systems. You'll learn how to build robust autonomous humanoid systems that can gracefully handle failures and recover from various error conditions.

## Overview

Robust error handling is crucial for autonomous humanoid systems operating in real-world environments. This section covers strategies for detecting, handling, and recovering from various types of errors that can occur in VLA systems.

## Types of Errors in VLA Systems

### 1. Sensor and Perception Errors

```python
class PerceptionErrorHandling:
    def __init__(self, node):
        self.node = node
        self.last_known_environment = {}
        self.confidence_threshold = 0.7

    def handle_perception_error(self, error_type, error_details):
        """
        Handle perception-related errors
        """
        if error_type == 'object_not_found':
            return self.handle_object_not_found(error_details)
        elif error_type == 'sensor_failure':
            return self.handle_sensor_failure(error_details)
        elif error_type == 'low_confidence_detection':
            return self.handle_low_confidence_detection(error_details)
        elif error_type == 'occlusion':
            return self.handle_occlusion(error_details)
        else:
            return self.handle_unknown_perception_error(error_details)

    def handle_object_not_found(self, details):
        """
        Handle when expected object is not found
        """
        target_object = details.get('target_object')
        search_location = details.get('search_location')

        self.node.get_logger().warn(f'Object {target_object} not found in {search_location}')

        # Try alternative locations
        alternative_locations = self.get_alternative_search_locations(
            target_object, search_location
        )

        for location in alternative_locations:
            self.node.get_logger().info(f'Trying alternative location: {location}')

            # Attempt to find object in alternative location
            search_result = self.search_for_object_at_location(
                target_object, location
            )

            if search_result['found']:
                return {
                    'success': True,
                    'object_location': search_result['location'],
                    'strategy': 'alternative_location_search'
                }

        # If still not found, ask for human assistance
        return self.request_human_assistance({
            'request': f'Could not find {target_object}. Can you help?',
            'options': ['Specify new location', 'Cancel task', 'Try different object']
        })

    def handle_sensor_failure(self, details):
        """
        Handle sensor failure with redundancy
        """
        failed_sensor = details.get('sensor_name')
        backup_sensors = self.get_backup_sensors(failed_sensor)

        for backup_sensor in backup_sensors:
            try:
                # Try to get data from backup sensor
                backup_data = self.get_sensor_data(backup_sensor)

                if backup_data:
                    return {
                        'success': True,
                        'data_source': backup_sensor,
                        'data': backup_data,
                        'strategy': 'sensor_redundancy'
                    }
            except Exception as e:
                self.node.get_logger().warn(f'Backup sensor {backup_sensor} also failed: {e}')
                continue

        # All sensors failed, return to safe state
        return self.return_to_safe_state({
            'error': f'All sensors failed for {failed_sensor}',
            'recommendation': 'Manual inspection required'
        })

    def handle_low_confidence_detection(self, details):
        """
        Handle detections with low confidence
        """
        detection = details.get('detection')
        confidence = details.get('confidence')

        if confidence < self.confidence_threshold:
            # Try to improve confidence through:
            # - Multiple sensor fusion
            # - Different viewing angles
            # - Enhanced lighting conditions

            improvement_strategies = [
                self.increase_sensor_resolution,
                self.change_viewing_angle,
                self.adjust_lighting_conditions
            ]

            for strategy in improvement_strategies:
                improved_result = strategy(detection)

                if improved_result and improved_result.get('confidence', 0) > self.confidence_threshold:
                    return {
                        'success': True,
                        'improved_detection': improved_result,
                        'confidence': improved_result['confidence'],
                        'strategy': 'confidence_improvement'
                    }

        # If confidence cannot be improved, use conservative approach
        return {
            'success': True,
            'detection': detection,
            'confidence': confidence,
            'conservative_approach': True,
            'risk_assessment': self.assess_risk_from_low_confidence(confidence)
        }

    def handle_occlusion(self, details):
        """
        Handle object occlusion
        """
        occluded_object = details.get('object')
        occluder = details.get('occluder')

        self.node.get_logger().info(f'{occluded_object} is occluded by {occluder}')

        # Strategies for dealing with occlusion
        strategies = [
            self.move_to_different_viewpoint,
            self.wait_for_movement,
            self.ask_for_help
        ]

        for strategy in strategies:
            result = strategy(occluded_object, details)

            if result['success']:
                return result

        # If all strategies fail, use conservative approach
        return {
            'success': False,
            'error': f'Could not handle occlusion of {occluded_object}',
            'conservative_approach': True
        }

    def get_alternative_search_locations(self, target_object, original_location):
        """
        Get alternative locations where object might be found
        """
        # This would use semantic knowledge about object placement
        # For example: cups are often in kitchen, office, dining room
        semantic_map = {
            'cup': ['kitchen', 'office', 'dining_room'],
            'book': ['office', 'living_room', 'bedroom'],
            'keys': ['hall', 'kitchen', 'office']
        }

        possible_locations = semantic_map.get(target_object, [])

        # Remove original location
        if original_location in possible_locations:
            possible_locations.remove(original_location)

        return possible_locations
```

### 2. Navigation Errors

```python
class NavigationErrorHandling:
    def __init__(self, node):
        self.node = node
        self.local_costmap = None
        self.global_costmap = None

    def handle_navigation_error(self, error_type, error_details):
        """
        Handle navigation-related errors
        """
        if error_type == 'obstacle_in_path':
            return self.handle_dynamic_obstacle(error_details)
        elif error_type == 'localization_lost':
            return self.handle_localization_loss(error_details)
        elif error_type == 'path_planning_failure':
            return self.handle_path_planning_failure(error_details)
        elif error_type == 'stuck_robot':
            return self.handle_robot_stuck(error_details)
        else:
            return self.handle_unknown_navigation_error(error_details)

    def handle_dynamic_obstacle(self, details):
        """
        Handle dynamic obstacles in navigation path
        """
        obstacle_position = details.get('obstacle_position')
        original_path = details.get('original_path')
        target_pose = details.get('target_pose')

        self.node.get_logger().info(f'Dynamic obstacle detected at {obstacle_position}')

        # Try to find alternative path around obstacle
        alternative_path = self.find_path_around_obstacle(
            original_path, obstacle_position, target_pose
        )

        if alternative_path:
            return {
                'success': True,
                'new_path': alternative_path,
                'strategy': 'dynamic_replanning'
            }

        # If no alternative path, wait for obstacle to move
        wait_result = self.wait_for_obstacle_clearance(obstacle_position)

        if wait_result['cleared']:
            # Retry original path
            return {
                'success': True,
                'original_path': original_path,
                'strategy': 'wait_and_retry'
            }

        # If obstacle persists, request human assistance
        return self.request_alternative_route(target_pose)

    def handle_localization_loss(self, details):
        """
        Handle when robot loses localization
        """
        self.node.get_logger().error('Robot lost localization')

        # Stop robot immediately
        self.emergency_stop()

        # Try to recover localization
        recovery_strategies = [
            self.attempt_relocalization,
            self.return_to_known_location,
            self.request_manual_repositioning
        ]

        for strategy in recovery_strategies:
            result = strategy()

            if result['success']:
                return result

        # If all strategies fail, return to safe state
        return self.return_to_safe_state({
            'error': 'Localization permanently lost',
            'recovery_attempts': 'all_failed'
        })

    def handle_path_planning_failure(self, details):
        """
        Handle when path planner cannot find a valid path
        """
        start_pose = details.get('start_pose')
        goal_pose = details.get('goal_pose')

        self.node.get_logger().warn(f'Could not find path from {start_pose} to {goal_pose}')

        # Try different planning strategies
        planning_strategies = [
            self.use_global_planner,
            self.use_local_planner,
            self.use_sampling_based_planner
        ]

        for strategy in planning_strategies:
            try:
                alternative_path = strategy(start_pose, goal_pose)

                if alternative_path:
                    return {
                        'success': True,
                        'path': alternative_path,
                        'strategy': strategy.__name__
                    }
            except Exception as e:
                self.node.get_logger().warn(f'Planning strategy {strategy.__name__} failed: {e}')
                continue

        # If all planning strategies fail, check if goal is unreachable
        goal_reachable = self.check_goal_reachability(goal_pose)

        if not goal_reachable:
            return {
                'success': False,
                'error': 'Goal is unreachable',
                'suggestion': 'Modify goal position or environment'
            }

        # Try to find closest reachable point
        closest_reachable = self.find_closest_reachable_point(goal_pose)

        return {
            'success': True,
            'closest_point': closest_reachable,
            'strategy': 'modified_goal'
        }

    def handle_robot_stuck(self, details):
        """
        Handle when robot becomes stuck
        """
        current_position = details.get('current_position')
        stuck_reason = details.get('reason')

        self.node.get_logger().error(f'Robot is stuck at {current_position}, reason: {stuck_reason}')

        # Emergency stop
        self.emergency_stop()

        # Recovery strategies
        recovery_strategies = [
            self.perform_rock_back_and_forth,
            self.rotate_in_place,
            self.request_manual_unsticking
        ]

        for strategy in recovery_strategies:
            result = strategy(stuck_reason)

            if result['success']:
                return result

        # If recovery fails, return to safe state
        return self.return_to_safe_state({
            'error': 'Robot stuck and recovery failed',
            'position': current_position
        })

    def find_path_around_obstacle(self, original_path, obstacle_pos, target_pose):
        """
        Find alternative path around obstacle
        """
        # Use local path planning to find way around obstacle
        # This would typically use a local planner like DWA or TEB
        pass

    def wait_for_obstacle_clearance(self, obstacle_position):
        """
        Wait for obstacle to move out of way
        """
        import time

        wait_duration = 30  # Wait up to 30 seconds
        check_interval = 1  # Check every second

        for i in range(0, wait_duration, check_interval):
            if not self.is_obstacle_present(obstacle_position):
                return {
                    'cleared': True,
                    'wait_time': i
                }

            time.sleep(check_interval)

        return {
            'cleared': False,
            'wait_time': wait_duration
        }
```

### 3. Manipulation Errors

```python
class ManipulationErrorHandling:
    def __init__(self, node):
        self.node = node
        self.gripper_controller = None
        self.arm_controller = None

    def handle_manipulation_error(self, error_type, error_details):
        """
        Handle manipulation-related errors
        """
        if error_type == 'grasp_failure':
            return self.handle_grasp_failure(error_details)
        elif error_type == 'object_slip':
            return self.handle_object_slip(error_details)
        elif error_type == 'collision_during_manipulation':
            return self.handle_manipulation_collision(error_details)
        elif error_type == 'gripper_failure':
            return self.handle_gripper_failure(error_details)
        elif error_type == 'joint_limit_violation':
            return self.handle_joint_limit_violation(error_details)
        else:
            return self.handle_unknown_manipulation_error(error_details)

    def handle_grasp_failure(self, details):
        """
        Handle when grasp attempt fails
        """
        target_object = details.get('object')
        grasp_attempt = details.get('grasp_attempt')
        failure_reason = details.get('failure_reason')

        self.node.get_logger().warn(f'Grasp failed for {target_object}, reason: {failure_reason}')

        # Try alternative grasp strategies
        alternative_grasps = self.generate_alternative_grasps(
            target_object, grasp_attempt
        )

        for alternative_grasp in alternative_grasps:
            self.node.get_logger().info(f'Trying alternative grasp: {alternative_grasp}')

            grasp_result = self.execute_grasp(alternative_grasp)

            if grasp_result['success']:
                return {
                    'success': True,
                    'grasp': alternative_grasp,
                    'strategy': 'alternative_grasp'
                }

        # If all grasps fail, assess object properties
        object_properties = self.assess_object_properties(target_object)

        if object_properties['is_too_small']:
            return {
                'success': False,
                'error': f'Object {target_object} is too small to grasp',
                'recommendation': 'Use different gripper or tool'
            }

        if object_properties['is_too_heavy']:
            return {
                'success': False,
                'error': f'Object {target_object} is too heavy to lift',
                'recommendation': 'Request assistance or use alternative method'
            }

        if object_properties['is_deformable']:
            return {
                'success': False,
                'error': f'Object {target_object} is deformable and hard to grasp',
                'recommendation': 'Use vacuum gripper or alternative method'
            }

        # If we can't determine the issue, request human assistance
        return self.request_human_assistance({
            'request': f'Could not grasp {target_object}. Need help.',
            'object': target_object,
            'attempts': len(alternative_grasps)
        })

    def handle_object_slip(self, details):
        """
        Handle when object slips from gripper during transport
        """
        object_name = details.get('object')
        grip_force = details.get('grip_force')
        slip_magnitude = details.get('slip_magnitude')

        self.node.get_logger().warn(f'Object {object_name} slipping, grip force: {grip_force}')

        # Increase grip force
        new_grip_force = min(grip_force * 1.5, self.max_grip_force)

        if self.adjust_grip_force(new_grip_force):
            # Re-assess the situation
            if not self.check_for_slip():
                return {
                    'success': True,
                    'grip_force': new_grip_force,
                    'strategy': 'increase_grip_force'
                }

        # If increasing grip force doesn't help, try different approach
        return self.use_tool_or_alternative_method(details)

    def handle_manipulation_collision(self, details):
        """
        Handle collisions during manipulation
        """
        collision_point = details.get('collision_point')
        affected_joints = details.get('affected_joints')
        collision_force = details.get('collision_force')

        self.node.get_logger().error(f'Manipulation collision at {collision_point}')

        # Emergency stop manipulation
        self.emergency_stop_manipulation()

        # Assess damage and adjust plan
        damage_assessment = self.assess_collision_damage(collision_point, collision_force)

        if damage_assessment['minor']:
            # Continue with adjusted plan
            adjusted_plan = self.adjust_plan_for_collision(
                original_plan=details.get('original_plan'),
                collision_point=collision_point
            )

            return {
                'success': True,
                'adjusted_plan': adjusted_plan,
                'strategy': 'collision_adjustment'
            }

        elif damage_assessment['major']:
            # Stop manipulation and return to safe state
            return self.return_to_safe_state({
                'error': 'Major collision during manipulation',
                'damage_assessment': damage_assessment
            })

        else:
            # Unknown severity, request human assessment
            return self.request_human_assessment({
                'collision_point': collision_point,
                'force': collision_force
            })

    def generate_alternative_grasps(self, target_object, failed_grasp):
        """
        Generate alternative grasp strategies
        """
        alternatives = []

        # Different grasp types
        grasp_types = ['top_grasp', 'side_grasp', 'pinch_grasp', 'power_grasp']

        for grasp_type in grasp_types:
            if grasp_type != failed_grasp.get('type'):
                alternative = self.plan_grasp_of_type(target_object, grasp_type)
                if alternative:
                    alternatives.append(alternative)

        # Different approach angles
        approach_angles = [0, 45, 90, 135, 180, 225, 270, 315]
        for angle in approach_angles:
            if angle != failed_grasp.get('approach_angle'):
                alternative = self.plan_grasp_with_approach_angle(target_object, angle)
                if alternative:
                    alternatives.append(alternative)

        return alternatives
```

### 4. LLM Planning Errors

```python
class LLMPlanningErrorHandling:
    def __init__(self, node):
        self.node = node
        self.llm_client = None

    def handle_llm_planning_error(self, error_type, error_details):
        """
        Handle LLM planning-related errors
        """
        if error_type == 'api_failure':
            return self.handle_api_failure(error_details)
        elif error_type == 'malformed_response':
            return self.handle_malformed_response(error_details)
        elif error_type == 'invalid_plan':
            return self.handle_invalid_plan(error_details)
        elif error_type == 'timeout':
            return self.handle_llm_timeout(error_details)
        elif error_type == 'context_window_exceeded':
            return self.handle_context_window_exceeded(error_details)
        else:
            return self.handle_unknown_llm_error(error_details)

    def handle_api_failure(self, details):
        """
        Handle LLM API failure
        """
        api_error = details.get('error')
        task_description = details.get('task')

        self.node.get_logger().error(f'LLM API failed: {api_error}')

        # Try fallback LLM provider
        fallback_providers = [
            self.call_openai_alternative,
            self.call_anthropic_claude,
            self.call_google_gemini
        ]

        for provider in fallback_providers:
            try:
                result = provider(task_description)

                if result and result.get('success'):
                    return {
                        'success': True,
                        'plan': result.get('plan'),
                        'provider': provider.__name__,
                        'strategy': 'fallback_provider'
                    }
            except Exception as e:
                self.node.get_logger().warn(f'Fallback provider {provider.__name__} failed: {e}')
                continue

        # If all providers fail, use rule-based fallback
        rule_based_plan = self.generate_rule_based_plan(task_description)

        if rule_based_plan:
            return {
                'success': True,
                'plan': rule_based_plan,
                'provider': 'rule_based',
                'strategy': 'rule_based_fallback'
            }

        # If everything fails, request human planning
        return self.request_human_planning(task_description)

    def handle_malformed_response(self, details):
        """
        Handle malformed LLM response
        """
        raw_response = details.get('raw_response')

        self.node.get_logger().warn(f'Malformed LLM response: {raw_response}')

        # Try to parse with more forgiving parser
        try:
            # Remove markdown formatting if present
            clean_response = self.clean_markdown_wrapping(raw_response)

            # Try to extract JSON
            extracted_json = self.extract_json_from_text(clean_response)

            if extracted_json:
                return {
                    'success': True,
                    'plan': extracted_json,
                    'strategy': 'malformed_response_recovery'
                }
        except Exception as e:
            self.node.get_logger().warn(f'Could not recover from malformed response: {e}')

        # If recovery fails, request re-generation
        return self.request_plan_regeneration(details.get('task'))

    def handle_invalid_plan(self, details):
        """
        Handle when LLM generates invalid plan
        """
        invalid_plan = details.get('plan')
        validation_errors = details.get('validation_errors')

        self.node.get_logger().warn(f'Invalid plan generated: {validation_errors}')

        # Provide feedback to LLM to improve
        feedback = self.generate_feedback_for_llm(invalid_plan, validation_errors)

        refined_plan = self.refine_plan_with_feedback(
            original_plan=invalid_plan,
            feedback=feedback
        )

        if refined_plan and self.validate_plan(refined_plan):
            return {
                'success': True,
                'plan': refined_plan,
                'strategy': 'plan_refinement'
            }

        # If refinement fails, use alternative approach
        return self.generate_plan_with_constraints(
            task=details.get('task'),
            constraints=self.extract_constraints_from_errors(validation_errors)
        )

    def clean_markdown_wrapping(self, text):
        """
        Remove markdown code block wrapping
        """
        if text.startswith('```'):
            # Find the first occurrence of '{' and last occurrence of '}'
            start = text.find('{')
            end = text.rfind('}') + 1

            if start != -1 and end != 0:
                return text[start:end]

        return text

    def extract_json_from_text(self, text):
        """
        Extract JSON from text that may contain other content
        """
        import re
        import json

        # Look for JSON-like patterns
        json_patterns = [
            r'\{.*\}',  # Objects
            r'\[.*\]',  # Arrays
        ]

        for pattern in json_patterns:
            matches = re.findall(pattern, text, re.DOTALL)

            for match in matches:
                try:
                    # Clean up the match
                    clean_match = match.strip()
                    if clean_match.startswith('{') or clean_match.startswith('['):
                        parsed = json.loads(clean_match)
                        return parsed
                except json.JSONDecodeError:
                    continue

        return None

    def generate_feedback_for_llm(self, invalid_plan, errors):
        """
        Generate feedback to help LLM improve
        """
        feedback = f"""
        The plan you generated has the following issues:
        {errors}

        Please regenerate the plan fixing these issues:
        1. Ensure all required fields are present
        2. Use valid action types
        3. Provide correct parameter types and ranges
        4. Ensure logical sequence of actions
        5. Maintain consistency between related actions

        Return valid JSON only.
        """

        return feedback
```

## System-Level Error Handling

### 1. Global Error Handler

```python
class GlobalErrorHandler:
    def __init__(self, node):
        self.node = node
        self.error_log = []
        self.emergency_stop_active = False
        self.safe_state = None

    def handle_error(self, error_type, error_details, severity='warning'):
        """
        Global error handler that coordinates response across all subsystems
        """
        # Log the error
        error_entry = {
            'timestamp': self.node.get_clock().now().nanoseconds,
            'type': error_type,
            'details': error_details,
            'severity': severity,
            'handled_by': None
        }

        self.error_log.append(error_entry)

        # Determine appropriate response based on severity
        if severity == 'critical':
            return self.handle_critical_error(error_type, error_details)
        elif severity == 'error':
            return self.handle_standard_error(error_type, error_details)
        elif severity == 'warning':
            return self.handle_warning(error_type, error_details)
        else:
            # Info level - just log
            return {'success': True, 'action': 'logged_only'}

    def handle_critical_error(self, error_type, error_details):
        """
        Handle critical errors that require immediate action
        """
        self.node.get_logger().fatal(f'CRITICAL ERROR: {error_type} - {error_details}')

        # Activate emergency stop
        self.emergency_stop()

        # Save current state for debugging
        self.save_debug_state()

        # Return to safe state
        safe_result = self.return_to_safe_state()

        # Notify human operators
        self.notify_operators({
            'type': 'critical_error',
            'error': error_type,
            'details': error_details,
            'action_taken': 'emergency_stop_and_safe_return'
        })

        return {
            'success': False,
            'error': error_type,
            'critical': True,
            'actions_taken': ['emergency_stop', 'return_to_safe_state', 'notification_sent']
        }

    def handle_standard_error(self, error_type, error_details):
        """
        Handle standard errors with recovery attempts
        """
        self.node.get_logger().error(f'ERROR: {error_type} - {error_details}')

        # Attempt recovery based on error type
        recovery_result = self.attempt_error_recovery(error_type, error_details)

        if recovery_result['success']:
            return recovery_result

        # If recovery fails, escalate
        return self.escalate_error(error_type, error_details)

    def handle_warning(self, error_type, error_details):
        """
        Handle warnings - usually just log and continue
        """
        self.node.get_logger().warn(f'WARNING: {error_type} - {error_details}')

        # Log warning but continue operation
        return {
            'success': True,
            'warning': error_type,
            'continuing_operation': True
        }

    def attempt_error_recovery(self, error_type, error_details):
        """
        Attempt to recover from error using appropriate strategy
        """
        # Map error types to recovery strategies
        recovery_strategies = {
            'navigation': self.recover_navigation_error,
            'perception': self.recover_perception_error,
            'manipulation': self.recover_manipulation_error,
            'communication': self.recover_communication_error,
            'llm': self.recover_llm_error
        }

        # Determine error category
        error_category = self.categorize_error(error_type)

        if error_category in recovery_strategies:
            strategy = recovery_strategies[error_category]
            return strategy(error_details)

        # If no specific strategy, use general recovery
        return self.general_error_recovery(error_type, error_details)

    def categorize_error(self, error_type):
        """
        Categorize error type for appropriate handling
        """
        error_type_lower = error_type.lower()

        if any(category in error_type_lower for category in ['navigate', 'path', 'move', 'go']):
            return 'navigation'
        elif any(category in error_type_lower for category in ['detect', 'see', 'find', 'percept']):
            return 'perception'
        elif any(category in error_type_lower for category in ['grasp', 'manipul', 'pick', 'place']):
            return 'manipulation'
        elif any(category in error_type_lower for category in ['comm', 'speak', 'talk']):
            return 'communication'
        elif any(category in error_type_lower for category in ['llm', 'plan', 'think']):
            return 'llm'
        else:
            return 'general'

    def emergency_stop(self):
        """
        Execute emergency stop procedure
        """
        if not self.emergency_stop_active:
            self.node.get_logger().warn('EMERGENCY STOP ACTIVATED')

            # Stop all robot motion
            self.stop_all_motion()

            # Deactivate all actuators
            self.deactivate_actuators()

            # Set emergency stop flag
            self.emergency_stop_active = True

    def return_to_safe_state(self):
        """
        Return robot to predefined safe state
        """
        self.node.get_logger().info('Returning to safe state...')

        # Move to safe position
        safe_position_result = self.move_to_safe_position()

        # Deactivate manipulators
        deactivate_result = self.deactivate_manipulators()

        # Set safe configuration
        config_result = self.set_safe_configuration()

        return {
            'success': safe_position_result['success'] and deactivate_result['success'] and config_result['success'],
            'actions': ['move_to_safe_position', 'deactivate_manipulators', 'set_safe_configuration']
        }

    def save_debug_state(self):
        """
        Save current system state for debugging
        """
        debug_info = {
            'timestamp': self.node.get_clock().now().nanoseconds,
            'robot_state': self.get_current_robot_state(),
            'sensor_data': self.get_current_sensor_data(),
            'recent_errors': self.error_log[-10:],  # Last 10 errors
            'system_status': self.get_system_status()
        }

        # Save to debug file
        import json
        import datetime

        filename = f"debug_state_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        filepath = f"/tmp/{filename}"

        with open(filepath, 'w') as f:
            json.dump(debug_info, f, indent=2)

        self.node.get_logger().info(f'Debug state saved to {filepath}')

    def notify_operators(self, notification):
        """
        Notify human operators of critical events
        """
        # This could send email, SMS, or publish to notification topic
        notification_msg = String()
        notification_msg.data = json.dumps(notification)

        # Publish to notification topic
        # self.notification_publisher.publish(notification_msg)

        self.node.get_logger().info(f'Operator notified: {notification}')
```

## Fallback Mechanisms

### 1. Hierarchical Fallback System

```python
class HierarchicalFallbackSystem:
    def __init__(self, node):
        self.node = node
        self.fallback_levels = [
            self.level_1_minimal_functionality,
            self.level_2_basic_operations,
            self.level_3_full_capability,
        ]

    def execute_with_fallbacks(self, task):
        """
        Execute task with hierarchical fallbacks
        """
        for level_idx, fallback_level in enumerate(self.fallback_levels):
            try:
                result = fallback_level(task)

                if result['success']:
                    return result

                self.node.get_logger().info(f'Level {level_idx + 1} failed, trying next level')

            except Exception as e:
                self.node.get_logger().warn(f'Level {level_idx + 1} threw exception: {e}')
                continue

        # All levels failed
        return {
            'success': False,
            'error': 'All fallback levels failed',
            'levels_tried': len(self.fallback_levels)
        }

    def level_1_minimal_functionality(self, task):
        """
        Level 1: Minimal functionality - basic communication only
        """
        if self.has_basic_communication():
            return {
                'success': True,
                'functionality': 'minimal',
                'operations': ['communication'],
                'status': 'operational'
            }

        return {'success': False}

    def level_2_basic_operations(self, task):
        """
        Level 2: Basic operations - navigation and simple actions
        """
        capabilities = []

        if self.has_navigation():
            capabilities.append('navigation')

        if self.has_basic_manipulation():
            capabilities.append('manipulation')

        if self.has_perception():
            capabilities.append('perception')

        if capabilities:
            return {
                'success': True,
                'functionality': 'basic',
                'operations': capabilities,
                'status': 'operational'
            }

        return {'success': False}

    def level_3_full_capability(self, task):
        """
        Level 3: Full capability - all systems operational
        """
        # This is the normal operating mode
        # Would call the full VLA pipeline
        pass

    def has_basic_communication(self):
        """
        Check if basic communication is available
        """
        # Check if speakers/microphones are functional
        return True  # Placeholder

    def has_navigation(self):
        """
        Check if navigation is available
        """
        # Check if navigation stack is running
        return True  # Placeholder

    def has_basic_manipulation(self):
        """
        Check if basic manipulation is available
        """
        # Check if manipulator joints are responsive
        return True  # Placeholder

    def has_perception(self):
        """
        Check if perception is available
        """
        # Check if cameras/sensors are functional
        return True  # Placeholder
```

### 2. Graceful Degradation

```python
class GracefulDegradationSystem:
    def __init__(self, node):
        self.node = node
        self.current_capability_level = 'full'
        self.capability_requirements = {
            'full': ['navigation', 'perception', 'manipulation', 'communication', 'llm_planning'],
            'high': ['navigation', 'perception', 'manipulation', 'communication'],
            'medium': ['navigation', 'perception', 'communication'],
            'basic': ['navigation', 'communication'],
            'minimal': ['communication']
        }

    def execute_task_with_degradation(self, task):
        """
        Execute task with graceful capability degradation
        """
        # Start with highest capability level
        available_capabilities = self.get_available_capabilities()

        # Determine highest possible level with available capabilities
        capability_level = self.determine_capability_level(available_capabilities)

        if capability_level == 'none':
            return self.handle_no_capabilities(task)

        # Adjust task based on available capabilities
        adjusted_task = self.adjust_task_for_capability_level(task, capability_level)

        # Execute with current capability level
        result = self.execute_task_at_level(adjusted_task, capability_level)

        if result['success']:
            return result

        # If execution fails, try lower capability level
        return self.attempt_degraded_execution(task, capability_level)

    def determine_capability_level(self, available_capabilities):
        """
        Determine current capability level based on available systems
        """
        for level, required_caps in self.capability_requirements.items():
            if all(cap in available_capabilities for cap in required_caps):
                return level

        return 'none'

    def adjust_task_for_capability_level(self, task, capability_level):
        """
        Adjust task to match current capability level
        """
        adjusted_task = task.copy()

        if capability_level in ['minimal', 'basic']:
            # Remove manipulation requirements
            adjusted_task['remove_manipulation'] = True

        if capability_level in ['minimal', 'basic', 'medium']:
            # Simplify navigation (avoid complex maneuvers)
            adjusted_task['simplified_navigation'] = True

        if capability_level == 'minimal':
            # Only communication possible
            adjusted_task['only_communication'] = True

        return adjusted_task

    def get_available_capabilities(self):
        """
        Check which capabilities are currently available
        """
        capabilities = []

        # Check each capability
        if self.is_navigation_available():
            capabilities.append('navigation')
        if self.is_perception_available():
            capabilities.append('perception')
        if self.is_manipulation_available():
            capabilities.append('manipulation')
        if self.is_communication_available():
            capabilities.append('communication')
        if self.is_llm_available():
            capabilities.append('llm_planning')

        return capabilities

    def is_navigation_available(self):
        """
        Check if navigation system is available
        """
        # Check if navigation stack is running and responsive
        return True  # Placeholder

    def is_perception_available(self):
        """
        Check if perception system is available
        """
        # Check if cameras and perception nodes are running
        return True  # Placeholder

    def is_manipulation_available(self):
        """
        Check if manipulation system is available
        """
        # Check if manipulator joints are responsive
        return True  # Placeholder

    def is_communication_available(self):
        """
        Check if communication system is available
        """
        # Check if speakers/microphones are functional
        return True  # Placeholder

    def is_llm_available(self):
        """
        Check if LLM planning is available
        """
        # Check if LLM API is accessible
        return True  # Placeholder

    def handle_no_capabilities(self, task):
        """
        Handle when no capabilities are available
        """
        self.node.get_logger().error('No capabilities available, requesting assistance')

        return {
            'success': False,
            'error': 'No operational capabilities',
            'recommendation': 'Manual intervention required'
        }
```

These error handling and fallback mechanisms provide comprehensive strategies for managing failures in Vision-Language-Action systems, ensuring that autonomous humanoid robots can operate safely and reliably even when encountering unexpected issues.