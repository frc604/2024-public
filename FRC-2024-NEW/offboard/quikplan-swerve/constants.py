import numpy as np

# TODO: verify if angles are correct

# Start positions
AMP_START_POS = (1.5, 6.0, np.pi)
MID_START_POS = (1.5, 5.0, np.pi)
SOURCE_START_POS = (1.5, 3.5, np.pi)

# Close pieces
AMP_CLOSE_PIECE_APPROACH = (2.0, 6.8, np.pi)
AMP_CLOSE_PIECE = (2.5, 6.8, np.pi)
MID_CLOSE_PIECE_APPROACH = (2.0, 5.5, np.pi)
MID_CLOSE_PIECE = (2.5, 5.5, np.pi)
SOURCE_CLOSE_PIECE_APPROACH = (2.0, 3.9, np.pi)
SOURCE_CLOSE_PIECE = (2.5, 3.9, np.pi)

# Mid line pieces, labeled 1-5 from amp side to source side
FIRST_MIDLINE_PIECE_APPROACH = (6.5, 7.2, np.pi)
FIRST_MIDLINE_PIECE = (8.0, 7.4, np.pi)
SECOND_MIDLINE_PIECE_APPROACH = (6.5, 5.5, np.pi)
SECOND_MIDLINE_PIECE = (8.0, 5.5, np.pi)
THIRD_MIDLINE_PIECE_APPROACH = (6.5, 4.0, np.pi)
THIRD_MIDLINE_PIECE = (8.0, 4.0, np.pi)
FOURTH_MIDLINE_PIECE_APPROACH = (7.0, 2.0, np.pi * 0.875)
FOURTH_MIDLINE_PIECE = (8.0, 2.3, np.pi * 0.875)
FIFTH_MIDLINE_PIECE_APPROACH = (6.5, 1.0, np.pi)
FIFTH_MIDLINE_PIECE = (8.0, 0.8, np.pi)

# Scoring
AMP_CLOSE_PIECE_SCORING = (2.5, 6.8, np.pi)
MID_CLOSE_PIECE_SCORING = (2.5, 5.5, np.pi)
SOURCE_CLOSE_PIECE_SCORING = (2.2, 3.8, np.pi)
FIRST_MIDLINE_PIECE_SCORING = (
    5.0,
    6.5,
    np.pi,
)  # TODO: find location where we shoot midline pieces from
FIRST_MIDLINE_PIECE_SCORING_CLOSE = (2.0, 6.0, np.pi)
SECOND_MIDLINE_PIECE_SCORING = (5.0, 6.5, np.pi)
THIRD_MIDLINE_PIECE_SCORING = (4.5, 5.2, np.pi)
FOURTH_MIDLINE_PIECE_SCORING = (4.0, 2.5, np.pi)
FIFTH_MIDLINE_PIECE_SCORING = (4.3, 2.5, np.pi)

# Intermediate waypoints
UNDER_STAGE_WAYPOINT = (5.7, 4.0, np.pi)
CLOSE_AMP_STAGE_WAYPOINT = (3.3, 5.0, np.pi)  # drive around the stage
CLOSE_SOURCE_STAGE_WAYPOINT = (3.3, 3.0, np.pi)
FAR_AMP_STAGE_WAYPOINT = (5.8, 6.2, np.pi)
FAR_SOURCE_STAGE_WAYPOINT = (5.8, 1.5, np.pi)
FIRST_MIDLINE_PIECE_INTERMEDIATE = (2.3, 5.95, np.pi)
