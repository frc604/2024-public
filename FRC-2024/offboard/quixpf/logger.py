import csv
from multiprocessing import Process, Manager, Queue
import os
import pickle


class Logger:
    def __init__(self, filename):
        dirname = os.path.dirname(__file__)
        self.filename = filename
        csvlogfile = open(
            os.path.join(dirname, self.filename + ".csv"), "w", newline=""
        )
        self.writer = csv.writer(csvlogfile)
        self.logfile = open(os.path.join(dirname, self.filename + ".log"), "ab")

    def init(self, num_particles, odometry, retroreflective_targets, apriltag_targets):
        pickle.dump(
            (num_particles, odometry, retroreflective_targets, apriltag_targets),
            self.logfile,
        )

    def log(self, time_elapsed, odometry, vision, best_estimate, has_vision):
        # Write simple data to CSV
        try:
            x, y, theta = best_estimate
            self.writer.writerow([time_elapsed, x, y, theta, has_vision])
        except Exception as e:
            print(e)

        # Write full data to log
        try:
            pickle.dump(
                (time_elapsed, odometry, vision, best_estimate, has_vision),
                self.logfile,
            )
        except Exception as e:
            print(e)
