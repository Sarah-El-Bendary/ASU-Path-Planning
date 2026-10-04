from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:

        import math

        # ---------------------------------------------------------
        # Extract the vehicle's current position and heading.
        # ---------------------------------------------------------

        cx = self.car_pose.x
        cy = self.car_pose.y
        yaw = self.car_pose.yaw

        # ---------------------------------------------------------
        # Separate detected cones according to their track-side color.
        # Blue cones define the LEFT boundary; yellow cones define the RIGHT boundary.
        # ---------------------------------------------------------

        blue_cones = [
            cone for cone in self.cones
            if cone.color == 1
        ]

        yellow_cones = [
            cone for cone in self.cones
            if cone.color == 0
        ]

        # =========================================================
        # CASE 1: NO CONES
        #
        # With no boundary information available, generate a straight path
        # using the vehicle's current heading.
        # =========================================================

        if not blue_cones and not yellow_cones:

            path = [(cx, cy)]

            for i in range(1, 21):

                distance = 0.5 * i

                x = (
                    cx +
                    math.cos(yaw) * distance
                )

                y = (
                    cy +
                    math.sin(yaw) * distance
                )

                path.append((x, y))

            return path

        # =========================================================
        # CASE 2: BOTH BLUE AND YELLOW CONES ARE VISIBLE
        #
        # Match cones from opposite boundaries and use their midpoints
        # to estimate the track centerline. If one boundary contains
        # additional cones, extend the centerline using the offset
        # determined from the matched pair.
        # =========================================================

        if blue_cones and yellow_cones:

            center_points = []

            used_blue = set()
            used_yellow = set()

            # -----------------------------------------------------
            # Associate each blue cone with the nearest available yellow cone.
            # -----------------------------------------------------

            for blue_i, blue in enumerate(blue_cones):

                best_yellow_i = -1
                best_yellow = None
                best_distance = float("inf")

                for yellow_i, yellow in enumerate(yellow_cones):

                    if yellow_i in used_yellow:
                        continue

                    distance = math.hypot(
                        blue.x - yellow.x,
                        blue.y - yellow.y
                    )

                    if distance < best_distance:

                        best_distance = distance
                        best_yellow_i = yellow_i
                        best_yellow = yellow

                if best_yellow is not None:

                    used_blue.add(blue_i)
                    used_yellow.add(best_yellow_i)

                    # The midpoint provides an estimate of the track center.
                    center_x = (
                        blue.x +
                        best_yellow.x
                    ) / 2.0

                    center_y = (
                        blue.y +
                        best_yellow.y
                    ) / 2.0

                    center_points.append(
                        (center_x, center_y)
                    )

            # -----------------------------------------------------
            # Determine the boundary-to-center offset from the first matched pair.
            # -----------------------------------------------------

            blue_offset_x = 0.0
            blue_offset_y = 0.0

            yellow_offset_x = 0.0
            yellow_offset_y = 0.0

            if used_blue and used_yellow:

                # Use the first matched pair as the reference for the center offset.
                blue_index = next(iter(used_blue))
                yellow_index = next(iter(used_yellow))

                blue = blue_cones[blue_index]
                yellow = yellow_cones[yellow_index]

                center_x, center_y = center_points[0]

                # Offset from the blue boundary to the estimated centerline.
                blue_offset_x = center_x - blue.x
                blue_offset_y = center_y - blue.y

                # Offset from the yellow boundary to the estimated centerline.
                yellow_offset_x = center_x - yellow.x
                yellow_offset_y = center_y - yellow.y

            # -----------------------------------------------------
            # Handle unmatched blue cones by applying the previously estimated
            # boundary-to-center offset.
            # -----------------------------------------------------

            for blue_i, blue in enumerate(blue_cones):

                if blue_i in used_blue:
                    continue

                if center_points:

                    center_x = (
                        blue.x +
                        blue_offset_x
                    )

                    center_y = (
                        blue.y +
                        blue_offset_y
                    )

                    center_points.append(
                        (center_x, center_y)
                    )

            # -----------------------------------------------------
            # Handle unmatched yellow cones by applying the previously estimated
            # boundary-to-center offset.
            # -----------------------------------------------------

            for yellow_i, yellow in enumerate(yellow_cones):

                if yellow_i in used_yellow:
                    continue

                if center_points:

                    center_x = (
                        yellow.x +
                        yellow_offset_x
                    )

                    center_y = (
                        yellow.y +
                        yellow_offset_y
                    )

                    center_points.append(
                        (center_x, center_y)
                    )

            # -----------------------------------------------------
            # Order centerline points from nearest to farthest relative to the car.
            # -----------------------------------------------------

            center_points.sort(
                key=lambda point:
                    math.hypot(
                        point[0] - cx,
                        point[1] - cy
                    )
            )

            # -----------------------------------------------------
            # Fall back to a straight path if no centerline points were generated.
            # -----------------------------------------------------

            if not center_points:

                path = [(cx, cy)]

                for i in range(1, 21):

                    distance = 0.5 * i

                    x = (
                        cx +
                        math.cos(yaw) * distance
                    )

                    y = (
                        cy +
                        math.sin(yaw) * distance
                    )

                    path.append((x, y))

                return path

            # -----------------------------------------------------
            # Compute the vehicle's forward unit direction from its heading.
            # -----------------------------------------------------

            forward_x = math.cos(yaw)
            forward_y = math.sin(yaw)

            # -----------------------------------------------------
            # Initialize the path at the vehicle's current position.
            # -----------------------------------------------------

            path = [(cx, cy)]

            first_x, first_y = center_points[0]

            dx = first_x - cx
            dy = first_y - cy

            distance_to_first = math.hypot(
                dx,
                dy
            )

            # -----------------------------------------------------
            # Smoothly transition from the vehicle's current heading to the centerline.
            # -----------------------------------------------------

            if distance_to_first > 0:

                control_distance = min(
                    1.5,
                    distance_to_first * 0.45
                )

                # P0 represents the current vehicle position.
                p0_x = cx
                p0_y = cy

                # P1 extends along the vehicle's current heading.
                p1_x = (
                    cx +
                    forward_x * control_distance
                )

                p1_y = (
                    cy +
                    forward_y * control_distance
                )

                # Compute the normalized direction from the car toward the first centerline point.
                target_x = (
                    dx /
                    distance_to_first
                )

                target_y = (
                    dy /
                    distance_to_first
                )

                p2_distance = min(
                    1.0,
                    distance_to_first * 0.30
                )

                # P2 approaches the first centerline point along the target direction.
                p2_x = (
                    first_x -
                    target_x * p2_distance
                )

                p2_y = (
                    first_y -
                    target_y * p2_distance
                )

                # P3 is the first centerline point.
                p3_x = first_x
                p3_y = first_y

                steps = max(
                    2,
                    math.ceil(
                        distance_to_first / 0.5
                    )
                )

                for i in range(1, steps + 1):

                    t = i / steps

                    # Evaluate the cubic Bezier curve at the current parameter.
                    x = (
                        (1 - t) ** 3 * p0_x
                        +
                        3 * (1 - t) ** 2 * t * p1_x
                        +
                        3 * (1 - t) * t ** 2 * p2_x
                        +
                        t ** 3 * p3_x
                    )

                    y = (
                        (1 - t) ** 3 * p0_y
                        +
                        3 * (1 - t) ** 2 * t * p1_y
                        +
                        3 * (1 - t) * t ** 2 * p2_y
                        +
                        t ** 3 * p3_y
                    )

                    path.append((x, y))

                    if len(path) >= 21:
                        return path[:21]

                        # -----------------------------------------------------
            # Follow the remaining centerline segments.
            # Straight interpolation is used for normal segments, while
            # sharp changes in direction are smoothed with a cubic Bezier curve.
            # -----------------------------------------------------

            for i in range(1, len(center_points)):

                start_x, start_y = center_points[i - 1]
                end_x, end_y = center_points[i]

                dx = end_x - start_x
                dy = end_y - start_y

                segment_length = math.hypot(dx, dy)

                if segment_length == 0:
                    continue

                # -------------------------------------------------
                # Compute the direction entering the current centerline point.
                # -------------------------------------------------

                if i == 1:

                    incoming_x = start_x - cx
                    incoming_y = start_y - cy

                else:

                    incoming_x = (
                        start_x -
                        center_points[i - 2][0]
                    )

                    incoming_y = (
                        start_y -
                        center_points[i - 2][1]
                    )

                incoming_length = math.hypot(
                    incoming_x,
                    incoming_y
                )

                # -------------------------------------------------
                # Compute the direction leaving the current centerline point.
                # -------------------------------------------------

                outgoing_x = dx
                outgoing_y = dy

                outgoing_length = segment_length

                if incoming_length > 0:

                    incoming_x /= incoming_length
                    incoming_y /= incoming_length

                if outgoing_length > 0:

                    outgoing_x /= outgoing_length
                    outgoing_y /= outgoing_length

                # -------------------------------------------------
                # Measure the angle between the incoming and outgoing directions.
                # -------------------------------------------------

                dot = (
                    incoming_x * outgoing_x +
                    incoming_y * outgoing_y
                )

                dot = max(-1.0, min(1.0, dot))

                turn_angle = math.acos(dot)

                # -------------------------------------------------
                # Apply curve smoothing only when the heading change exceeds
                # the selected sharp-turn threshold, preserving normal segments.
                # -------------------------------------------------

                if turn_angle > math.radians(75.0):

                    # Set the Bezier control-handle length for the turn.
                    handle = min(
                        segment_length * 0.45,
                        0.8
                    )

                    p0_x = start_x
                    p0_y = start_y

                    p1_x = (
                        start_x +
                        incoming_x * handle
                    )

                    p1_y = (
                        start_y +
                        incoming_y * handle
                    )

                    p2_x = (
                        end_x -
                        outgoing_x * handle
                    )

                    p2_y = (
                        end_y -
                        outgoing_y * handle
                    )

                    p3_x = end_x
                    p3_y = end_y

                    steps = max(
                        4,
                        math.ceil(
                            segment_length / 0.25
                        )
                    )

                    for j in range(1, steps + 1):

                        t = j / steps

                        x = (
                            (1 - t) ** 3 * p0_x
                            +
                            3 * (1 - t) ** 2 * t * p1_x
                            +
                            3 * (1 - t) * t ** 2 * p2_x
                            +
                            t ** 3 * p3_x
                        )

                        y = (
                            (1 - t) ** 3 * p0_y
                            +
                            3 * (1 - t) ** 2 * t * p1_y
                            +
                            3 * (1 - t) * t ** 2 * p2_y
                            +
                            t ** 3 * p3_y
                        )

                        path.append((x, y))

                        if len(path) >= 21:
                            return path[:21]

                else:

                    # -------------------------------------------------
                    # Normal segment: keep the original straight path.
                    # -------------------------------------------------

                    steps = max(
                        1,
                        math.ceil(
                            segment_length / 0.5
                        )
                    )

                    for j in range(1, steps + 1):

                        t = j / steps

                        x = (
                            start_x +
                            dx * t
                        )

                        y = (
                            start_y +
                            dy * t
                        )

                        path.append((x, y))

                        if len(path) >= 21:
                            return path[:21]

            # -----------------------------------------------------
            # Extend the path using the direction of the final centerline segment.
            # -----------------------------------------------------

            if len(center_points) >= 2:

                previous_x, previous_y = center_points[-2]
                last_x, last_y = center_points[-1]

                direction_x = (
                    last_x -
                    previous_x
                )

                direction_y = (
                    last_y -
                    previous_y
                )

            else:

                last_x, last_y = center_points[-1]

                direction_x = (
                    last_x -
                    cx
                )

                direction_y = (
                    last_y -
                    cy
                )

            direction_length = math.hypot(
                direction_x,
                direction_y
            )

            if direction_length > 0:

                direction_x /= direction_length
                direction_y /= direction_length

            else:

                direction_x = forward_x
                direction_y = forward_y

            current_x, current_y = path[-1]

            while len(path) < 21:

                current_x += direction_x * 0.5
                current_y += direction_y * 0.5

                path.append(
                    (current_x, current_y)
                )

            return path[:21]

                # =========================================================
        # CASE 3: ONLY ONE SIDE OF THE TRACK IS VISIBLE
        #
        # Blue   = LEFT boundary
        # Yellow = RIGHT boundary
        #
        # Track width assumption = 4 m.
        # Therefore the centerline is approximately 2 m
        # from the visible boundary.
        #
        # Blue   -> center is 2 m to the RIGHT
        # Offset from the yellow boundary to the estimated centerline. is 2 m to the LEFT
        # =========================================================

        TRACK_WIDTH = 4.0
        CENTER_OFFSET = TRACK_WIDTH / 2.0

        # ---------------------------------------------------------
        # Compute the vehicle's forward direction.
        # ---------------------------------------------------------

        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)

        # ---------------------------------------------------------
        # Treat all supplied cones as observations of the same visible boundary.
        # ---------------------------------------------------------

        relevant_cones = list(self.cones)

        # ---------------------------------------------------------
        # Fall back to a straight path if no cones are available.
        # ---------------------------------------------------------

        if not relevant_cones:

            path = [(cx, cy)]

            for i in range(1, 21):

                distance = 0.5 * i

                x = (
                    cx +
                    forward_x * distance
                )

                y = (
                    cy +
                    forward_y * distance
                )

                path.append((x, y))

            return path

        visible_color = relevant_cones[0].color

        # ---------------------------------------------------------
        # Estimate the direction of the visible boundary from the cone geometry.
        # ---------------------------------------------------------

        if len(relevant_cones) >= 2:

            max_distance = -1.0

            first_cone = relevant_cones[0]
            last_cone = relevant_cones[1]

            for i in range(len(relevant_cones)):

                for j in range(i + 1, len(relevant_cones)):

                    dx = (
                        relevant_cones[j].x -
                        relevant_cones[i].x
                    )

                    dy = (
                        relevant_cones[j].y -
                        relevant_cones[i].y
                    )

                    distance = math.hypot(
                        dx,
                        dy
                    )

                    if distance > max_distance:

                        max_distance = distance

                        first_cone = relevant_cones[i]
                        last_cone = relevant_cones[j]

            boundary_dx = (
                last_cone.x -
                first_cone.x
            )

            boundary_dy = (
                last_cone.y -
                first_cone.y
            )

            boundary_length = math.hypot(
                boundary_dx,
                boundary_dy
            )

            if boundary_length > 0:

                boundary_x = (
                    boundary_dx /
                    boundary_length
                )

                boundary_y = (
                    boundary_dy /
                    boundary_length
                )

            else:

                boundary_x = forward_x
                boundary_y = forward_y

        else:

            # With only one cone, estimate the local boundary direction using the car heading.

            boundary_x = forward_x
            boundary_y = forward_y

            first_cone = relevant_cones[0]

        # ---------------------------------------------------------
        # Orient the estimated boundary direction toward the observed cones.
        # ---------------------------------------------------------

        reference_cone = relevant_cones[0]

        cone_vector_x = (
            reference_cone.x -
            cx
        )

        cone_vector_y = (
            reference_cone.y -
            cy
        )

        direction_dot = (
            boundary_x * cone_vector_x
            +
            boundary_y * cone_vector_y
        )

        if direction_dot < 0:

            boundary_x = -boundary_x
            boundary_y = -boundary_y

        # ---------------------------------------------------------
        # Compute unit normal directions on the left and right sides of the boundary.
        # ---------------------------------------------------------

        left_x = -boundary_y
        left_y = boundary_x

        right_x = boundary_y
        right_y = -boundary_x

        # ---------------------------------------------------------
        # Generate virtual centerline points at the assumed track-center offset.
        # ---------------------------------------------------------

        center_points = []

        for cone in relevant_cones:

            if visible_color == 1:

                # -------------------------------------------------
                # BLUE = LEFT boundary: shift inward by 2 m toward the RIGHT.
                # -------------------------------------------------

                center_x = (
                    cone.x +
                    right_x * CENTER_OFFSET
                )

                center_y = (
                    cone.y +
                    right_y * CENTER_OFFSET
                )

            else:

                # -------------------------------------------------
                # YELLOW = RIGHT boundary: shift inward by 2 m toward the LEFT.
                # -------------------------------------------------

                center_x = (
                    cone.x +
                    left_x * CENTER_OFFSET
                )

                center_y = (
                    cone.y +
                    left_y * CENTER_OFFSET
                )

            center_points.append(
                (center_x, center_y)
            )

        # ---------------------------------------------------------
        # Order the estimated centerline points along the visible boundary.
        # ---------------------------------------------------------

        reference_x = relevant_cones[0].x
        reference_y = relevant_cones[0].y

        center_points.sort(
            key=lambda point:
                (
                    (point[0] - reference_x) * boundary_x
                    +
                    (point[1] - reference_y) * boundary_y
                )
        )

        # ---------------------------------------------------------
        # Initialize the path at the exact vehicle position.
        # ---------------------------------------------------------

        path = [(cx, cy)]

        first_x, first_y = center_points[0]

        dx = first_x - cx
        dy = first_y - cy

        distance_to_first = math.hypot(
            dx,
            dy
        )

        # ---------------------------------------------------------
        # SMOOTH INITIAL TRANSITION
        #
        # Begin with the vehicle's current heading and gradually transition
        # toward the estimated centerline using a cubic Bezier curve.
        # ---------------------------------------------------------

        if distance_to_first > 0:

            # -----------------------------------------------------
            # P0 represents the current vehicle position. position
            # -----------------------------------------------------

            p0_x = cx
            p0_y = cy

            # -----------------------------------------------------
            # P1 extends along the vehicle's current heading. A longer control
            # distance produces a smoother initial transition.
            # -----------------------------------------------------

            heading_control_distance = min(
                1.5,
                distance_to_first * 0.55
            )

            p1_x = (
                cx +
                forward_x * heading_control_distance
            )

            p1_y = (
                cy +
                forward_y * heading_control_distance
            )

            # -----------------------------------------------------
            # Compute the normalized direction toward the first centerline point.
            # -----------------------------------------------------

            target_x = (
                dx /
                distance_to_first
            )

            target_y = (
                dy /
                distance_to_first
            )

            # -----------------------------------------------------
            # P2 controls the gradual approach to the first centerline point.
            # -----------------------------------------------------

            target_control_distance = min(
                1.0,
                distance_to_first * 0.30
            )

            p2_x = (
                first_x -
                target_x * target_control_distance
            )

            p2_y = (
                first_y -
                target_y * target_control_distance
            )

            # -----------------------------------------------------
            # P3 is the first estimated centerline point.
            # -----------------------------------------------------

            p3_x = first_x
            p3_y = first_y

            # -----------------------------------------------------
            # Generate sufficient samples to maintain the required point spacing.
            # -----------------------------------------------------

            steps = max(
                6,
                math.ceil(
                    distance_to_first / 0.3
                )
            )

            for i in range(1, steps + 1):

                t = i / steps

                # Evaluate the cubic Bezier equation at the current parameter.

                x = (
                    (1 - t) ** 3 * p0_x
                    +
                    3 * (1 - t) ** 2 * t * p1_x
                    +
                    3 * (1 - t) * t ** 2 * p2_x
                    +
                    t ** 3 * p3_x
                )

                y = (
                    (1 - t) ** 3 * p0_y
                    +
                    3 * (1 - t) ** 2 * t * p1_y
                    +
                    3 * (1 - t) * t ** 2 * p2_y
                    +
                    t ** 3 * p3_y
                )

                path.append(
                    (x, y)
                )

                if len(path) >= 21:
                    return path[:21]

        # ---------------------------------------------------------
        # Follow the remaining estimated centerline points.
        # ---------------------------------------------------------

        for i in range(1, len(center_points)):

            start_x, start_y = center_points[i - 1]
            end_x, end_y = center_points[i]

            dx = end_x - start_x
            dy = end_y - start_y

            segment_length = math.hypot(
                dx,
                dy
            )

            if segment_length == 0:
                continue

            steps = max(
                1,
                math.ceil(
                    segment_length / 0.5
                )
            )

            for j in range(1, steps + 1):

                t = j / steps

                x = (
                    start_x +
                    dx * t
                )

                y = (
                    start_y +
                    dy * t
                )

                path.append(
                    (x, y)
                )

                if len(path) >= 21:
                    return path[:21]

        # ---------------------------------------------------------
        # Extend the path along the final estimated centerline direction.
        # ---------------------------------------------------------

        if len(center_points) >= 2:

            previous_x, previous_y = center_points[-2]
            last_x, last_y = center_points[-1]

            direction_x = (
                last_x -
                previous_x
            )

            direction_y = (
                last_y -
                previous_y
            )

        else:

            direction_x = (
                center_points[0][0] -
                cx
            )

            direction_y = (
                center_points[0][1] -
                cy
            )

        direction_length = math.hypot(
            direction_x,
            direction_y
        )

        if direction_length > 0:

            direction_x /= direction_length
            direction_y /= direction_length

        else:

            direction_x = forward_x
            direction_y = forward_y

        current_x, current_y = path[-1]

        while len(path) < 21:

            current_x += direction_x * 0.5
            current_y += direction_y * 0.5

            path.append(
                (current_x, current_y)
            )

        return path[:21]  