#!/usr/bin/env python3
"""
ALGORITHM SELECTION
===================

The search algorithm uses LIDAR to map walls and does a 3m GRID SEARCH.

IED detector has 3m range, so visiting points on a 3m grid = full coverage.

Strategy:
1. LIDAR maps walls -> identifies free space
2. Divide free space into 3m grid cells
3. Visit each grid cell systematically
4. Mark cells as SEARCHED when visited
5. Always go to nearest UNSEARCHED cell
6. Return to entrance before 7 minutes (battery limit)
"""

ALGORITHM = "search_systematic_mapper"

ALGORITHM_INFO = {
    "search_systematic_mapper": {
        "name": "Efficient Grid V11",
        "description": "Based on V9 (92% coverage). 3m grid + rush mode for efficiency.",
        "recommended": True
    }
}
