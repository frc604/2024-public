from multiprocessing import Process
import os

import run_partial_231456
import run_partial_24531
import run_partial_25431
import run_partial_876
import run_partial_678
import run_partial_1456
import run_partial_1546


if __name__ == "__main__":
    # Delete old plots and CSVs
    dirname = os.path.dirname(__file__)
    for item in os.listdir(os.path.join(dirname, "plots/partial")):
        if item.endswith(".png"):
            os.remove(os.path.join(dirname, "plots/partial", item))
    for item in os.listdir(os.path.join(dirname, "../../src/main/deploy/partial")):
        if item.endswith(".csv"):
            os.remove(os.path.join(dirname, "../../src/main/deploy/partial", item))

    Process(target=run_partial_231456.plan_2s3s1s, args=(True,)).start()
    Process(target=run_partial_231456.plan_4s5, args=(True,)).start()
    Process(target=run_partial_231456.plan_5s6, args=(True,)).start()
    Process(target=run_partial_231456.plan_14, args=(True,)).start()
    Process(target=run_partial_231456.plan_45, args=(True,)).start()
    Process(target=run_partial_231456.plan_56, args=(True,)).start()
    Process(target=run_partial_231456.plan_6s, args=(True,)).start()
    Process(target=run_partial_231456.plan_4s5, args=(True,)).start()

    Process(target=run_partial_24531.plan_2s4, args=(True,)).start()
    Process(target=run_partial_24531.plan_4s5, args=(True,)).start()
    Process(target=run_partial_24531.plan_45, args=(True,)).start()
    Process(target=run_partial_24531.plan_5s3s1s, args=(True,)).start()

    Process(target=run_partial_25431.plan_2s5, args=(True,)).start()
    Process(target=run_partial_25431.plan_5s4, args=(True,)).start()
    Process(target=run_partial_25431.plan_54, args=(True,)).start()
    Process(target=run_partial_25431.plan_4s3s1s, args=(True,)).start()

    Process(target=run_partial_876.plan_8, args=(True,)).start()
    Process(target=run_partial_876.plan_8s7, args=(True,)).start()
    Process(target=run_partial_876.plan_87, args=(True,)).start()
    Process(target=run_partial_876.plan_7s6, args=(True,)).start()
    Process(target=run_partial_876.plan_76, args=(True,)).start()
    Process(target=run_partial_876.plan_6s, args=(True,)).start()

    Process(target=run_partial_678.plan_6, args=(True,)).start()
    Process(target=run_partial_678.plan_6s7, args=(True,)).start()
    Process(target=run_partial_678.plan_67, args=(True,)).start()
    Process(target=run_partial_678.plan_7s8, args=(True,)).start()
    Process(target=run_partial_678.plan_78, args=(True,)).start()
    Process(target=run_partial_678.plan_8s, args=(True,)).start()

    Process(target=run_partial_1456.plan_1s4, args=(True,)).start()
    Process(target=run_partial_1456.plan_4s5, args=(True,)).start()
    Process(target=run_partial_1456.plan_45, args=(True,)).start()
    Process(target=run_partial_1456.plan_5s6, args=(True,)).start()
    Process(target=run_partial_1456.plan_56, args=(True,)).start()
    Process(target=run_partial_1456.plan_6s, args=(True,)).start()

    Process(target=run_partial_1546.plan_1s5, args=(True,)).start()
    Process(target=run_partial_1546.plan_5s4, args=(True,)).start()
    Process(target=run_partial_1546.plan_54, args=(True,)).start()
    Process(target=run_partial_1546.plan_4s6, args=(True,)).start()
    Process(target=run_partial_1546.plan_46, args=(True,)).start()
    Process(target=run_partial_1546.plan_6s, args=(True,)).start()
