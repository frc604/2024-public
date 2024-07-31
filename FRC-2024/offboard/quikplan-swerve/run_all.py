from multiprocessing import Process
import os

from run_simple_test import plan as plan_simple
from run_square_test import plan as plan_square
from run_crazy_test import plan as plan_crazy
from run_23145 import plan as plan_23145
from run_23154 import plan as plan_23154
from run_876 import plan as plan_876
from run_25613 import plan as plan_25613


if __name__ == "__main__":
    # Delete old plots and CSVs
    dirname = os.path.dirname(__file__)
    for item in os.listdir(os.path.join(dirname, "plots")):
        if item.endswith(".png"):
            os.remove(os.path.join(dirname, "plots", item))
    for item in os.listdir(os.path.join(dirname, "../../src/main/deploy")):
        if item.endswith(".csv"):
            os.remove(os.path.join(dirname, "../../src/main/deploy", item))

    # Test plans
    Process(target=plan_simple, args=(True,)).start()
    Process(target=plan_square, args=(True,)).start()
    Process(target=plan_crazy, args=(True,)).start()

    # Auto plans
    Process(target=plan_23145, args=(True,)).start()
    Process(target=plan_23154, args=(True,)).start()
    Process(target=plan_876, args=(True,)).start()
    Process(target=plan_25613, args=(True,)).start()
