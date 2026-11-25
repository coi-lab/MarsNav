"""Elevation grids, slope calculations, and route planning in local meters."""

from heapq import heappop, heappush
from math import ceil, floor, hypot

import numpy as np


def block_mean(elevation, factor):
    rows, cols = elevation.shape
    if factor < 1 or rows % factor or cols % factor:
        raise ValueError("Crop dimensions must be divisible by the grid factor.")
    blocks = elevation.reshape(rows // factor, factor, cols // factor, factor)
    return blocks.mean(axis=(1, 3))


def slope_degrees(elevation, spacing):
    """Use centered differences inside and one-sided differences at edges."""
    dz_drow, dz_dcol = np.gradient(elevation, spacing)
    return np.degrees(np.arctan(np.hypot(dz_drow, dz_dcol)))


def astar(slope, spacing, start, goal, limit, weight):
    """Find the lowest-cost route between cells, or return None."""
    rows, cols = slope.shape
    allowed = slope < limit
    if not (allowed[start] and allowed[goal]):
        return None

    directions = ((-1, 0), (0, -1), (0, 1), (1, 0),
                  (-1, -1), (-1, 1), (1, -1), (1, 1))
    queue = [(0.0, 0.0, start)]
    costs = {start: 0.0}
    previous = {}
    while queue:
        _, cost, current = heappop(queue)
        if cost > costs[current]:
            continue
        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(previous[path[-1]])
            return list(reversed(path)), cost

        row, col = current
        for dr, dc in directions:
            neighbor = nr, nc = row + dr, col + dc
            if not (0 <= nr < rows and 0 <= nc < cols and allowed[neighbor]):
                continue
            if dr and dc and not (allowed[row, nc] and allowed[nr, col]):
                continue
            mean_slope = (slope[current] + slope[neighbor]) / 2
            step = spacing * hypot(dr, dc) * (1 + weight * mean_slope / limit)
            candidate = cost + step
            if candidate >= costs.get(neighbor, float("inf")):
                continue
            costs[neighbor] = candidate
            previous[neighbor] = current
            remaining = spacing * hypot(goal[0] - nr, goal[1] - nc)
            heappush(queue, (candidate + remaining, candidate, neighbor))
    return None


def route_metrics(points, native_hazards, native_spacing):
    """Measure the line length in each 1 m reference cell it crosses."""
    length = 0.0
    hazardous_length = 0.0
    for start, end in zip(points, points[1:]):
        segment_length = hypot(end[0] - start[0], end[1] - start[1])
        length += segment_length
        cuts = {0.0, 1.0}
        for first, last in zip(start, end):
            lo, hi = sorted((first, last))
            for index in range(floor(lo / native_spacing) + 1, ceil(hi / native_spacing)):
                cuts.add((index * native_spacing - first) / (last - first))
        cuts = sorted(cuts)
        for left, right in zip(cuts, cuts[1:]):
            midpoint = (left + right) / 2
            x, y = (a + midpoint * (b - a) for a, b in zip(start, end))
            if native_hazards[floor(y / native_spacing), floor(x / native_spacing)]:
                hazardous_length += (right - left) * segment_length
    return length, hazardous_length


def route_separation(first, second):
    """Approximate the symmetric maximum distance between two route lines."""
    def directed(source, target):
        vertices = np.asarray(target)
        origins = vertices[:-1]
        vectors = np.diff(vertices, axis=0)
        sizes = np.sum(vectors ** 2, axis=1)
        if not len(origins):
            origins = vertices
            vectors = np.zeros_like(vertices)
            sizes = np.zeros(len(vertices))
        sizes[sizes == 0] = 1
        samples = [source[0]]
        for start, end in zip(source, source[1:]):
            steps = max(1, ceil(hypot(end[0] - start[0], end[1] - start[1])))
            samples.extend(np.linspace(start, end, steps + 1)[1:])
        max_distance_squared = 0.0
        for offset in range(0, len(samples), 128):
            delta = np.asarray(samples[offset:offset + 128])[:, None, :] - origins
            fractions = np.clip(np.sum(delta * vectors, axis=2) / sizes, 0, 1)
            distances_squared = np.sum((delta - fractions[:, :, None] * vectors) ** 2, axis=2)
            max_distance_squared = max(max_distance_squared, distances_squared.min(axis=1).max())
        return float(np.sqrt(max_distance_squared))

    return max(directed(first, second), directed(second, first))
