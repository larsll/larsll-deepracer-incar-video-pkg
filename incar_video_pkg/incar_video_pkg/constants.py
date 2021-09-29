'''This module houses the constants for scripts package in simulation application'''
from enum import Enum
import cv2

MAIN_CAMERA_TOPIC="/video_mjpeg"
VIDEO_STATE_SRV="/media_state"
IMU_TOPIC="/imu_msg/raw"
PUBLISH_SENSOR_TOPIC="/video/sensor_stream"
PUBLISH_VIDEO_TOPIC="/video/mp4_stream"

class Mp4Parameter(Enum):
    """
    Describes the parameters used to save Mp4
    Extends:
        Enum
    """
    FOURCC = cv2.VideoWriter_fourcc(*'mp4v')
    FPS = 15
    FRAME_SIZE = (640, 480)


class RaceCarColorToRGB(Enum):
    """ Color to RGB mapping
    Extends:
        Enum
    """
    Black = (26, 26, 26)
    Grey = (135, 149, 150)
    Blue = (68, 95, 229)
    Red = (224, 26, 37)
    Orange = (255, 160, 10)
    White = (255, 255, 255)
    Purple = (159, 42, 195)


# Amount of time (in seconds) to wait, in order to prevent model state from
# spamming logs while the model is loading
WAIT_TO_PREVENT_SPAM = 2

# Radius of the circle plotted on the agent in pixels
RACECAR_CIRCLE_RADIUS = 30

# Decrease the minor image by a scale of provided number
SCALE_RATIO = 2.5


# Image size
class IconographicImageSize(Enum):
    """ The images that are provided are not accurate.
    Also some of the image pixels requires even number for the math.

    Attributes:
        FULL_IMAGE_SIZE: This is the same size that is given out by camera
        BOT_CAR_IMAGE_SIZE: Making the size rectangular and even pixel. Enlarging it.
        OBSTACLE_IMAGE_SIZE: [description]
        AGENTS_IMAGE_SIZE: [description]
        RACE_COMPLETE_IMAGE_SIZE: [description]
    """
    FULL_IMAGE_SIZE = (640, 480)
    BOT_CAR_IMAGE_SIZE = (int(34//SCALE_RATIO), int(34//SCALE_RATIO))
    OBSTACLE_IMAGE_SIZE = (int(34//SCALE_RATIO), int(34//SCALE_RATIO))
    AGENTS_IMAGE_SIZE = (int(88//SCALE_RATIO), int(88//SCALE_RATIO))
    VIRTUAL_EVENT_AGENTS_IMAGE_SIZE = (int(66 // SCALE_RATIO), int(66 // SCALE_RATIO))
    RACE_COMPLETE_IMAGE_SIZE = (308, 40)

# Track iconography png enums
class TrackAssetsIconographicPngs(Enum):
    """ Track images enum mapping

    Attributes:
        AGENTS_PNG: Different agents color images
        BOTS_PNG: Image of the bot shown in graphinology
        OBSTACLES_PNG: Image of the obstacle shown in graphinology
        OBSTACLE_OVERLAY_PNG: Gradient for the obstacle
        HEAD_TO_HEAD_OVERLAY_PNG: Gradient for the head to head
        RACE_COMPLETE_OVERLAY_PNG: Shown when the race is complete
    """
    AGENTS_PNG = ["DRL_video_racer1", "DRL_video_racer2"]
    VIRTUAL_EVENT_AGENTS_PNG = ["virtual_event_racer1", "virtual_event_racer2"]
    BOTS_PNG = "DRL_video_bot"
    OBSTACLES_PNG = "DRL_video_obstacles"
    OBSTACLE_OVERLAY_PNG = "DRL_video_oa_overlay"
    HEAD_TO_HEAD_OVERLAY_PNG = "DRL_video_h2h_overlay"
    RACE_COMPLETE_OVERLAY_PNG = "DRL_video_racecomplete_overlay"
    HEAD_TO_HEAD_OVERLAY_PNG_LEAGUE_LEADERBOARD = "DRL_video_h2h_overlay_league_leaderboard"
    OBSTACLE_OVERLAY_PNG_LEAGUE_LEADERBOARD = "DRL_video_oa_overlay_league_leaderboard"

# virtual event png
class VirtualEventIconographicPngs(Enum):
    OVERLAY_PNG = "virtual_event_overlay"
    SET = "virtual_event_set"
    GO = "virtual_event_go"
    FINISH = "virtual_event_finish"
    FINAL_FADING_IMAGE_50ALPHA = "final_fading_image_50alpha"


class XYPixelLoc(Enum):
    """ The mp4 image size is (480, 640). Rendering text at different locations
    """
    MULTI_AGENT_DISPLAY_NAME_LOC = [(10, 10), (450, 10)]
    MULTI_AGENT_EVAL_TIME = (240, 10)
    SINGLE_AGENT_DISPLAY_NAME_LOC = (10, 10)
    TIME_LOC = (10, 410)
    
    ACCEL_X_LOC = (10, 435)
    ACCEL_Y_LOC = (100, 435)
    ACCEL_Z_LOC = (190, 435)

    ANGULAR_X_LOC = (10, 455)
    ANGULAR_Y_LOC = (100, 455)
    ANGULAR_Z_LOC = (190, 455)

    RACE_TYPE_RACE_LOC = (10, 455)
    AWS_DEEPRACER_WATER_MARK_LOC = (445, 450)
    TRAINING_PHASE_LOC = (40, 400)
    TRACK_IMG_WITH_OFFSET_LOC = (0, 20)
    TRACK_IMG_WITHOUT_OFFSET_LOC = (0, 0)


# Race completion flag y-offset
RACE_COMPLETE_Y_OFFSET = 180

# Agent Video editor constants
MAX_FRAMES_IN_QUEUE = 2700
KVS_PUBLISH_PERIOD = 1.0/15.0
QUEUE_WAIT_TIME = 10 # In seconds
