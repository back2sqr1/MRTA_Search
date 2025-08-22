#!/usr/bin/env python3
"""
Test script for the enhanced SRQL parser with location pins
"""

import sys
sys.path.append('/home/david/ros2_ws/src/bdd_generation')

import sparser

def test_parser():
    # Read the updated SRQL file
    with open('/home/david/ros2_ws/src/bdd_generation/examples_2/tate-ex1b.srql', 'r') as f:
        content = f.read()
    
    print("Testing SRQL parser with location pins...")
    print("=" * 50)
    
    try:
        # Parse the file
        result = sparser.parse_world(content)
        
        print("Parsing successful!")
        print("\nLocations with coordinates:")
        print("-" * 30)
        
        for loc in result['locations']:
            if isinstance(loc, dict):
                if loc['x'] is not None and loc['y'] is not None:
                    print(f"{loc['name']} at ({loc['x']}, {loc['y']})")
                else:
                    print(f"{loc['name']} (no coordinates)")
            else:
                print(f"{loc} (old format)")
                
        print(f"\nTotal locations: {len(result['locations'])}")
        print(f"Properties: {result['properties']}")
        
        # Display robot information
        if 'robots' in result:
            robots = result['robots']
            print(f"\nRobots:")
            print(f"  Number of robots: {robots['num_robots']}")
            if robots['robot_starts']:
                print("  Robot starting locations:")
                for robot in robots['robot_starts']:
                    print(f"    {robot['robot_id']} starts at {robot['start_location']}")
            else:
                print("  No robot starting locations specified")
        
        return True
        
    except Exception as e:
        print(f"Parsing failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    test_parser()
