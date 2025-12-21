---
sidebar_position: 600
title: Validation Techniques for Generated Action Plans
---

# Validation Techniques for Generated Action Plans

This section covers various techniques for validating LLM-generated action plans before execution to ensure they are safe, feasible, and appropriate for the target robotic system.

## Overview

Validation is critical for ensuring that LLM-generated action plans are:
- **Safe**: Won't cause harm to people, property, or the robot
- **Feasible**: Can be executed with the robot's capabilities
- **Appropriate**: Suitable for the current context and environment
- **Logical**: Steps follow a sensible sequence

## 1. Static Plan Validation

Validate plans without executing them, analyzing the structure and content:

```python
class StaticPlanValidator:
    def __init__(self, robot_capabilities, environment_map):
        self.robot_capabilities = robot_capabilities
        self.environment_map = environment_map

    def validate_plan(self, plan):
        """
        Validate a complete action plan statically
        """
        validation_results = {
            'valid': True,
            'errors': [],
            'warnings': [],
            'suggestions': []
        }

        # Check if plan is empty
        if not plan or len(plan) == 0:
            validation_results['valid'] = False
            validation_results['errors'].append("Plan is empty")
            return validation_results

        # Validate each step
        for i, step in enumerate(plan):
            step_result = self.validate_step(step, i)
            validation_results['errors'].extend(step_result['errors'])
            validation_results['warnings'].extend(step_result['warnings'])
            validation_results['suggestions'].extend(step_result['suggestions'])

        # Check plan structure
        structure_result = self.validate_structure(plan)
        validation_results['errors'].extend(structure_result['errors'])
        validation_results['warnings'].extend(structure_result['warnings'])

        # Check safety constraints
        safety_result = self.validate_safety_constraints(plan)
        validation_results['errors'].extend(safety_result['errors'])
        validation_results['warnings'].extend(safety_result['warnings'])

        validation_results['valid'] = len(validation_results['errors']) == 0

        return validation_results

    def validate_step(self, step, step_index):
        """
        Validate individual plan step
        """
        result = {'errors': [], 'warnings': [], 'suggestions': []}

        # Check required fields
        if 'type' not in step:
            result['errors'].append(f"Step {step_index}: Missing 'type' field")
            return result

        action_type = step['type']

        # Check if action type is supported
        if action_type not in self.robot_capabilities['supported_actions']:
            result['errors'].append(f"Step {step_index}: Unsupported action type '{action_type}'")
            return result

        # Validate action-specific parameters
        param_result = self.validate_action_parameters(step, step_index)
        result['errors'].extend(param_result['errors'])
        result['warnings'].extend(param_result['warnings'])

        return result

    def validate_action_parameters(self, step, step_index):
        """
        Validate action-specific parameters
        """
        result = {'errors': [], 'warnings': []}
        action_type = step['type']
        params = step.get('parameters', {})

        # Define required parameters for each action type
        required_params = {
            'navigate': ['x', 'y'],
            'manipulate': ['action', 'object'],
            'perceive': ['target'],
            'communicate': ['message']
        }

        if action_type in required_params:
            for param in required_params[action_type]:
                if param not in params:
                    result['errors'].append(f"Step {step_index}: Missing required parameter '{param}' for action '{action_type}'")

        # Validate parameter values
        if action_type == 'navigate':
            if not self.is_valid_coordinate(params.get('x')) or not self.is_valid_coordinate(params.get('y')):
                result['errors'].append(f"Step {step_index}: Invalid coordinates for navigation")

        elif action_type == 'manipulate':
            valid_actions = ['grasp', 'release', 'move', 'pick_up', 'place']
            if params.get('action') not in valid_actions:
                result['errors'].append(f"Step {step_index}: Invalid manipulation action '{params.get('action')}'")

        return result

    def validate_structure(self, plan):
        """
        Validate plan structure and sequence
        """
        result = {'errors': [], 'warnings': []}

        # Check for duplicate consecutive actions
        prev_action = None
        for i, step in enumerate(plan):
            current_action = step.get('type')
            if current_action and current_action == prev_action:
                result['warnings'].append(f"Step {i}: Duplicate action '{current_action}' following previous step")

            prev_action = current_action

        # Check for logical sequence issues
        self.check_logical_sequence(plan, result)

        return result

    def check_logical_sequence(self, plan, result):
        """
        Check for logical issues in action sequence
        """
        for i in range(len(plan) - 1):
            current_step = plan[i]
            next_step = plan[i + 1]

            current_action = current_step.get('type')
            next_action = next_step.get('type')

            # Example: Check if manipulation is preceded by navigation
            if next_action == 'manipulate' and current_action != 'navigate':
                result['warnings'].append(f"Step {i+1}: Manipulation action may need preceding navigation to object location")

    def is_valid_coordinate(self, coord):
        """
        Check if coordinate is valid
        """
        if coord is None:
            return False
        try:
            coord = float(coord)
            # Check if coordinate is within reasonable bounds
            return -1000 <= coord <= 1000  # Reasonable limits for robot workspace
        except (TypeError, ValueError):
            return False

    def validate_safety_constraints(self, plan):
        """
        Validate safety constraints for the entire plan
        """
        result = {'errors': [], 'warnings': []}

        for i, step in enumerate(plan):
            action_type = step.get('type')

            if action_type == 'navigate':
                x = step.get('parameters', {}).get('x')
                y = step.get('parameters', {}).get('y')

                if x is not None and y is not None:
                    # Check if destination is in safe area
                    if not self.is_safe_destination(x, y):
                        result['errors'].append(f"Step {i}: Navigation to unsafe destination ({x}, {y})")

            elif action_type == 'manipulate':
                action = step.get('parameters', {}).get('action')
                if action == 'grasp':
                    # Additional safety checks for grasping
                    pass

        return result

    def is_safe_destination(self, x, y):
        """
        Check if destination is in safe area
        """
        # In a real system, this would check against environment map
        # and safety zones
        return True  # Placeholder implementation
```

## 2. Semantic Validation

Validate plans using semantic understanding and knowledge bases:

```python
class SemanticPlanValidator:
    def __init__(self):
        # Knowledge base of physical world constraints
        self.knowledge_base = {
            'object_properties': {
                'fragile': ['glass', 'cup', 'plate', 'mirror'],
                'heavy': ['table', 'chair', 'refrigerator'],
                'small': ['pen', 'key', 'coin']
            },
            'location_constraints': {
                'kitchen': ['food_items', 'cooking_utensils'],
                'bedroom': ['clothing', 'bedding'],
                'office': ['documents', 'electronics']
            },
            'common_sense_rules': [
                {
                    'condition': 'manipulate_fragile_without_care',
                    'description': 'Fragile objects should be handled carefully',
                    'severity': 'warning'
                },
                {
                    'condition': 'navigate_to_impossible_location',
                    'description': 'Cannot navigate to non-existent locations',
                    'severity': 'error'
                }
            ]
        }

    def validate_with_semantics(self, plan):
        """
        Validate plan using semantic knowledge
        """
        validation_results = {
            'valid': True,
            'errors': [],
            'warnings': [],
            'semantic_issues': []
        }

        for i, step in enumerate(plan):
            semantic_result = self.check_semantic_constraints(step, i)
            validation_results['errors'].extend(semantic_result['errors'])
            validation_results['warnings'].extend(semantic_result['warnings'])
            validation_results['semantic_issues'].extend(semantic_result['issues'])

        validation_results['valid'] = len(validation_results['errors']) == 0
        return validation_results

    def check_semantic_constraints(self, step, step_index):
        """
        Check semantic constraints for a single step
        """
        result = {'errors': [], 'warnings': [], 'issues': []}

        action_type = step.get('type')
        params = step.get('parameters', {})

        if action_type == 'manipulate':
            object_name = params.get('object', '').lower()
            action = params.get('action', '').lower()

            # Check if fragile object is handled appropriately
            if self.is_fragile_object(object_name) and action in ['grasp', 'pick_up']:
                result['warnings'].append(
                    f"Step {step_index}: Fragile object '{object_name}' should be handled carefully"
                )

            # Check if heavy object is being manipulated
            if self.is_heavy_object(object_name) and action in ['grasp', 'pick_up']:
                result['warnings'].append(
                    f"Step {step_index}: Heavy object '{object_name}' may exceed robot's lifting capacity"
                )

        elif action_type == 'navigate':
            # Check if navigation is to a reasonable location
            x, y = params.get('x'), params.get('y')
            if x is not None and y is not None:
                if not self.is_reasonable_destination(x, y):
                    result['warnings'].append(
                        f"Step {step_index}: Navigation to unusual location ({x}, {y})"
                    )

        return result

    def is_fragile_object(self, object_name):
        """
        Check if object is fragile
        """
        for fragile_item in self.knowledge_base['object_properties']['fragile']:
            if fragile_item in object_name:
                return True
        return False

    def is_heavy_object(self, object_name):
        """
        Check if object is heavy
        """
        for heavy_item in self.knowledge_base['object_properties']['heavy']:
            if heavy_item in object_name:
                return True
        return False

    def is_reasonable_destination(self, x, y):
        """
        Check if destination is reasonable
        """
        # Check if coordinates are extremely far away
        # In a real system, this would check against environment map
        return abs(x) <= 50 and abs(y) <= 50  # Reasonable workspace limits
```

## 3. Simulation-Based Validation

Validate plans by simulating their execution in a simulated environment:

```python
class SimulationValidator:
    def __init__(self, simulation_env):
        self.sim_env = simulation_env

    def validate_by_simulation(self, plan):
        """
        Validate plan by simulating execution
        """
        validation_results = {
            'valid': True,
            'errors': [],
            'warnings': [],
            'simulation_log': [],
            'success_probability': 0.0
        }

        try:
            # Reset simulation to initial state
            self.sim_env.reset()

            # Execute plan in simulation
            success_count = 0
            total_runs = 5  # Run multiple simulations with slight variations

            for run in range(total_runs):
                run_result = self.simulate_plan_execution(plan, run)
                validation_results['simulation_log'].append(run_result['log'])

                if run_result['success']:
                    success_count += 1

                # Collect any issues found
                validation_results['errors'].extend(run_result['errors'])
                validation_results['warnings'].extend(run_result['warnings'])

            # Calculate success probability
            validation_results['success_probability'] = success_count / total_runs

            # Plan is valid if majority of simulations succeed
            validation_results['valid'] = validation_results['success_probability'] >= 0.6

        except Exception as e:
            validation_results['valid'] = False
            validation_results['errors'].append(f"Simulation validation failed: {str(e)}")

        return validation_results

    def simulate_plan_execution(self, plan, run_number):
        """
        Simulate plan execution for one run
        """
        result = {
            'success': True,
            'errors': [],
            'warnings': [],
            'log': []
        }

        robot_state = self.sim_env.get_initial_state()

        for i, step in enumerate(plan):
            try:
                # Simulate step execution
                step_result = self.simulate_step(step, robot_state, i)

                if not step_result['success']:
                    result['success'] = False
                    result['errors'].extend(step_result['errors'])
                    result['log'].append(f"Step {i} failed: {step_result['errors']}")
                    break  # Stop simulation if step fails

                result['log'].append(f"Step {i} succeeded: {step_result['description']}")
                robot_state = step_result['new_state']

            except Exception as e:
                result['success'] = False
                result['errors'].append(f"Step {i} simulation error: {str(e)}")
                break

        return result

    def simulate_step(self, step, current_state, step_index):
        """
        Simulate execution of a single step
        """
        action_type = step.get('type')
        params = step.get('parameters', {})

        # Check if action is physically possible
        if not self.is_action_possible(action_type, params, current_state):
            return {
                'success': False,
                'errors': [f"Step {step_index}: Action not possible with current state"],
                'description': 'Action impossibility detected',
                'new_state': current_state
            }

        # Check for collisions
        if action_type == 'navigate':
            path = self.calculate_navigation_path(current_state['position'], params)
            if self.has_collision(path):
                return {
                    'success': False,
                    'errors': [f"Step {step_index}: Collision detected on navigation path"],
                    'description': 'Collision avoidance validation',
                    'new_state': current_state
                }

        # Update state based on action
        new_state = self.update_state_with_action(current_state, step)

        return {
            'success': True,
            'errors': [],
            'warnings': [],
            'description': f"Step {step_index} executed successfully",
            'new_state': new_state
        }

    def is_action_possible(self, action_type, params, state):
        """
        Check if action is physically possible
        """
        # Implementation would check robot capabilities vs action requirements
        return True  # Placeholder

    def calculate_navigation_path(self, start_pos, params):
        """
        Calculate navigation path
        """
        # In a real implementation, this would use path planning algorithms
        return []  # Placeholder

    def has_collision(self, path):
        """
        Check if path has collisions
        """
        # In a real implementation, this would check against environment obstacles
        return False  # Placeholder

    def update_state_with_action(self, current_state, step):
        """
        Update robot state based on action
        """
        # Update state based on action effects
        new_state = current_state.copy()
        # Apply action effects to state
        return new_state
```

## 4. Multi-Agent Validation

Validate plans considering multiple agents and their interactions:

```python
class MultiAgentValidator:
    def __init__(self, environment_model):
        self.env_model = environment_model

    def validate_multi_agent_plan(self, plans_by_agent):
        """
        Validate plans for multiple agents simultaneously
        """
        validation_results = {
            'valid': True,
            'conflicts': [],
            'coordination_issues': [],
            'resource_contentions': []
        }

        # Check for conflicts between agents
        conflicts = self.find_inter_agent_conflicts(plans_by_agent)
        validation_results['conflicts'] = conflicts

        # Check resource contention
        resource_issues = self.find_resource_contention(plans_by_agent)
        validation_results['resource_contentions'] = resource_issues

        # Check coordination requirements
        coordination_issues = self.find_coordination_issues(plans_by_agent)
        validation_results['coordination_issues'] = coordination_issues

        # Plan is valid if no critical conflicts exist
        validation_results['valid'] = (
            len([c for c in conflicts if c['severity'] == 'critical']) == 0 and
            len(resource_issues) == 0
        )

        return validation_results

    def find_inter_agent_conflicts(self, plans_by_agent):
        """
        Find conflicts between different agents' plans
        """
        conflicts = []

        agent_ids = list(plans_by_agent.keys())
        for i in range(len(agent_ids)):
            for j in range(i + 1, len(agent_ids)):
                agent1_id, agent2_id = agent_ids[i], agent_ids[j]
                plan1 = plans_by_agent[agent1_id]
                plan2 = plans_by_agent[agent2_id]

                # Check for spatial conflicts
                spatial_conflicts = self.find_spatial_conflicts(plan1, plan2, agent1_id, agent2_id)
                conflicts.extend(spatial_conflicts)

                # Check for temporal conflicts
                temporal_conflicts = self.find_temporal_conflicts(plan1, plan2, agent1_id, agent2_id)
                conflicts.extend(temporal_conflicts)

        return conflicts

    def find_spatial_conflicts(self, plan1, plan2, agent1_id, agent2_id):
        """
        Find spatial conflicts between two plans
        """
        conflicts = []

        # Check if both agents plan to occupy same space at same time
        for i, step1 in enumerate(plan1):
            if step1.get('type') == 'navigate':
                pos1 = (step1.get('parameters', {}).get('x'), step1.get('parameters', {}).get('y'))

                for j, step2 in enumerate(plan2):
                    if step2.get('type') == 'navigate':
                        pos2 = (step2.get('parameters', {}).get('x'), step2.get('parameters', {}).get('y'))

                        if pos1 and pos2 and self.are_positions_conflicting(pos1, pos2):
                            conflicts.append({
                                'type': 'spatial_collision',
                                'agents': [agent1_id, agent2_id],
                                'steps': [i, j],
                                'positions': [pos1, pos2],
                                'severity': 'critical',
                                'description': f"Agents {agent1_id} and {agent2_id} will occupy same position"
                            })

        return conflicts

    def are_positions_conflicting(self, pos1, pos2):
        """
        Check if two positions conflict (too close)
        """
        if not pos1 or not pos2:
            return False

        x1, y1 = pos1
        x2, y2 = pos2

        if x1 is None or y1 is None or x2 is None or y2 is None:
            return False

        # Calculate distance between positions
        distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
        min_safe_distance = 1.0  # meters

        return distance < min_safe_distance
```

## 5. Context-Aware Validation

Validate plans based on current context and environment:

```python
class ContextAwareValidator:
    def __init__(self, robot_state, environment_state):
        self.robot_state = robot_state
        self.env_state = environment_state

    def validate_with_context(self, plan):
        """
        Validate plan considering current context
        """
        validation_results = {
            'valid': True,
            'context_violations': [],
            'feasibility_issues': [],
            'context_specific_warnings': []
        }

        # Check if plan is appropriate for current time
        time_check = self.check_time_appropriateness(plan)
        validation_results['context_specific_warnings'].extend(time_check['warnings'])

        # Check if plan is appropriate for current environment
        env_check = self.check_environment_appropriateness(plan)
        validation_results['context_specific_warnings'].extend(env_check['warnings'])

        # Check resource availability
        resource_check = self.check_resource_availability(plan)
        validation_results['feasibility_issues'].extend(resource_check['issues'])

        # Check safety in current context
        safety_check = self.check_context_safety(plan)
        validation_results['context_violations'].extend(safety_check['violations'])

        validation_results['valid'] = (
            len(validation_results['context_violations']) == 0 and
            len([issue for issue in validation_results['feasibility_issues'] if issue['severity'] == 'critical']) == 0
        )

        return validation_results

    def check_time_appropriateness(self, plan):
        """
        Check if plan is appropriate for current time
        """
        result = {'warnings': []}

        current_hour = self.get_current_hour()

        # Check if navigation is planned during restricted hours
        for step in plan:
            if step.get('type') == 'navigate':
                # Example: avoid loud navigation at night
                if 22 <= current_hour <= 6:  # Night hours
                    result['warnings'].append(
                        f"Night hour navigation may disturb residents"
                    )

        return result

    def check_environment_appropriateness(self, plan):
        """
        Check if plan is appropriate for current environment
        """
        result = {'warnings': []}

        current_location = self.robot_state.get('location', 'unknown')

        for step in plan:
            if step.get('type') == 'communicate':
                message = step.get('parameters', {}).get('message', '').lower()

                # Check if communication is appropriate for location
                if current_location == 'library' and 'speak_loudly' in message:
                    result['warnings'].append(
                        f"Speaking loudly inappropriate in library environment"
                    )

        return result

    def check_resource_availability(self, plan):
        """
        Check if required resources are available
        """
        result = {'issues': []}

        # Check battery level for long navigation plans
        battery_level = self.robot_state.get('battery_level', 100)
        estimated_battery_usage = self.estimate_battery_usage(plan)

        if battery_level < estimated_battery_usage:
            result['issues'].append({
                'type': 'battery_insufficient',
                'estimated_usage': estimated_battery_usage,
                'current_level': battery_level,
                'severity': 'critical',
                'description': 'Insufficient battery for plan execution'
            })

        return result

    def check_context_safety(self, plan):
        """
        Check safety in current context
        """
        result = {'violations': []}

        # Check if humans are nearby for navigation actions
        humans_nearby = self.env_state.get('humans_nearby', 0)

        for step in plan:
            if step.get('type') == 'navigate':
                if humans_nearby > 0:
                    # Add safety checks for navigation near humans
                    result['violations'].append({
                        'type': 'human_safety',
                        'description': 'Navigation near humans requires safety checks',
                        'severity': 'warning'
                    })

        return result

    def estimate_battery_usage(self, plan):
        """
        Estimate battery usage for plan
        """
        usage = 0.0

        for step in plan:
            action_type = step.get('type')
            if action_type == 'navigate':
                # Estimate based on distance
                params = step.get('parameters', {})
                x, y = params.get('x', 0), params.get('y', 0)
                distance = (x**2 + y**2)**0.5
                usage += distance * 0.1  # 0.1% per meter
            elif action_type == 'manipulate':
                usage += 2.0  # Manipulation uses ~2% battery

        return usage

    def get_current_hour(self):
        """
        Get current hour
        """
        import datetime
        return datetime.datetime.now().hour
```

## 6. Continuous Validation During Execution

Validate plan execution in real-time:

```python
class RuntimeValidator:
    def __init__(self, robot_interface):
        self.robot_interface = robot_interface
        self.execution_context = {}

    def validate_during_execution(self, plan, step_index, current_state):
        """
        Validate plan during execution
        """
        validation_result = {
            'continue_execution': True,
            'modifications_needed': [],
            'errors': [],
            'warnings': []
        }

        # Check if conditions still hold
        if not self.preconditions_still_valid(plan[step_index], current_state):
            validation_result['continue_execution'] = False
            validation_result['errors'].append("Preconditions no longer valid")

        # Check if environment has changed significantly
        if self.environment_changed_significantly(current_state):
            validation_result['warnings'].append("Environment changed, consider replanning")

        # Check robot health
        robot_health = self.check_robot_health()
        if not robot_health['ok']:
            validation_result['continue_execution'] = False
            validation_result['errors'].append(f"Robot health issue: {robot_health['issues']}")

        # Check if goal is still relevant
        if not self.goal_still_relevant(plan, step_index):
            validation_result['warnings'].append("Goal may no longer be relevant")

        return validation_result

    def preconditions_still_valid(self, step, current_state):
        """
        Check if preconditions for next step are still met
        """
        action_type = step.get('type')

        if action_type == 'manipulate':
            # Check if object is still available
            target_object = step.get('parameters', {}).get('object')
            if target_object and not self.object_still_available(target_object, current_state):
                return False

        return True

    def environment_changed_significantly(self, current_state):
        """
        Check if environment has changed significantly
        """
        # Compare current state with expected state
        # This would involve sensor data comparison
        return False  # Placeholder

    def check_robot_health(self):
        """
        Check robot health status
        """
        # Check battery, motor status, sensor status, etc.
        health_status = {
            'ok': True,
            'issues': []
        }

        # Example checks
        battery_level = self.robot_interface.get_battery_level()
        if battery_level < 10:
            health_status['ok'] = False
            health_status['issues'].append(f"Battery critically low: {battery_level}%")

        return health_status

    def goal_still_relevant(self, plan, current_step_index):
        """
        Check if goal is still relevant
        """
        # This might involve checking for changes in user commands
        # or environmental conditions that affect the goal
        return True  # Placeholder

    def object_still_available(self, object_name, current_state):
        """
        Check if object is still available for manipulation
        """
        # Check current perception data
        detected_objects = current_state.get('detected_objects', [])
        return object_name in detected_objects
```

## 7. Validation Integration Pipeline

Combine multiple validation techniques:

```python
class ComprehensivePlanValidator:
    def __init__(self, robot_caps, env_map, sim_env):
        self.static_validator = StaticPlanValidator(robot_caps, env_map)
        self.semantic_validator = SemanticPlanValidator()
        self.simulation_validator = SimulationValidator(sim_env)
        self.context_validator = ContextAwareValidator(None, None)  # Will be set during validation
        self.runtime_validator = RuntimeValidator(None)  # Will be set during validation

    def validate_comprehensive(self, plan, current_context=None):
        """
        Perform comprehensive validation using multiple techniques
        """
        all_results = {}

        # Static validation
        all_results['static'] = self.static_validator.validate_plan(plan)

        # Semantic validation
        all_results['semantic'] = self.semantic_validator.validate_with_semantics(plan)

        # Simulation validation
        all_results['simulation'] = self.simulation_validator.validate_by_simulation(plan)

        # Context validation (if context provided)
        if current_context:
            context_val = ContextAwareValidator(
                current_context.get('robot_state', {}),
                current_context.get('environment_state', {})
            )
            all_results['context'] = context_val.validate_with_context(plan)

        # Overall validation result
        overall_valid = self.combine_validation_results(all_results)
        all_results['overall'] = overall_valid

        return all_results

    def combine_validation_results(self, validation_results):
        """
        Combine results from multiple validation techniques
        """
        combined = {
            'valid': True,
            'errors': [],
            'warnings': [],
            'confidence': 1.0
        }

        # Aggregate all errors and warnings
        for validator_name, results in validation_results.items():
            if isinstance(results, dict):
                if 'errors' in results:
                    combined['errors'].extend(results['errors'])
                if 'warnings' in results:
                    combined['warnings'].extend(results['warnings'])

        # Calculate overall validity
        combined['valid'] = len(combined['errors']) == 0

        # Calculate confidence based on validation results
        total_checks = len(validation_results)
        passed_checks = sum(
            1 for results in validation_results.values()
            if isinstance(results, dict) and results.get('valid', True)
        )
        combined['confidence'] = passed_checks / total_checks if total_checks > 0 else 0.0

        return combined

# Example usage
def validate_llm_generated_plan(plan, robot_capabilities, environment_map, simulation_env):
    """
    Validate an LLM-generated plan using comprehensive validation
    """
    validator = ComprehensivePlanValidator(robot_capabilities, environment_map, simulation_env)

    validation_results = validator.validate_comprehensive(plan)

    # Check if plan is acceptable
    overall_result = validation_results.get('overall', {})

    if overall_result.get('valid', False) and overall_result.get('confidence', 0) > 0.7:
        print("Plan validated successfully with high confidence")
        return True, validation_results
    else:
        print("Plan validation failed or low confidence")
        return False, validation_results
```

These validation techniques ensure that LLM-generated action plans are safe, feasible, and appropriate before execution on robotic systems, preventing potential issues and ensuring reliable operation.