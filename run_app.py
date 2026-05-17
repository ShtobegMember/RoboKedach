import sys
import os
import traceback
from base_station import pc_main

if __name__ == "__main__":
    # 1. Add the project root to the Python path
    # This ensures that imports work correctly regardless of the CWD.
    project_root = os.path.abspath(os.path.dirname(__file__))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    try:
        # 2. Execute the main entry point
        pc_main.main()
    except Exception as e:
        # 3. Catch and log any initialization errors that were being swallowed
        print("\n" + "!" * 60)
        print("CRITICAL BOOT ERROR:")
        print(f"Details: {e}")
        print("-" * 60)
        traceback.print_exc()
        print("!" * 60 + "\n")
        sys.exit(1)