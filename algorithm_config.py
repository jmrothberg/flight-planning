#!/usr/bin/env python3
"""
ALGORITHM SELECTION
===================

The search algorithm uses LiDAR to map walls and does a 2m GRID SEARCH.

IED detector has 2m range, so visiting points on a 2m grid = full coverage.

Strategy:
1. LiDAR maps walls -> identifies free space
2. Divide free space into 2m grid cells
3. Visit each grid cell systematically
4. Mark cells as SEARCHED when visited
5. Always go to nearest UNSEARCHED cell
6. Return to entrance before 7 minutes (battery limit)
"""

ALGORITHM = "search_systematic_mapper"

ALGORITHM_INFO = {
    "search_systematic_mapper": {
        "name": "Efficient Grid V11.9",
        "description": "2m grid + frontier/richness scoring + rush mode.",
        "recommended": True
    }
}
