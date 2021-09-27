""" Image editing class for head to bot, time-trail, obstacle where
there is only single agent
"""
import datetime
import logging
import cv2

from rclpy.time import Time

from sensor_msgs.msg import Imu

from incar_video_pkg.logger import Logger

from incar_video_pkg import utils
from incar_video_pkg.constants import (RaceCarColorToRGB, TrackAssetsIconographicPngs, IconographicImageSize,
                                  RACE_TYPE_TO_VIDEO_TEXT_MAPPING, XYPixelLoc, Mp4Parameter)
from incar_video_pkg.image_editing_interface import ImageEditingInterface

LOG = Logger(__name__, logging.INFO).get_logger()

class ImageEditing(ImageEditingInterface):
    """ Image editing class for head to bot, time-trail, obstacle where
    there is only single agent
    """
    def __init__(self, racecar_name, racecar_info, race_type):
        """ Initializing the required data for the head to bot, time-trail. This is used for single agent
        Arguments:
            racecars_info (list): list of dict having information of the agent
            race_type (str): Since this class is reused for all the different race_type
        """

        self.race_type = race_type
        self.racecar_name = racecar_name
        self.racecar_index = 0
        self.start_time = 0

        # Store the font which we will use to write the phase with
        self.amazon_ember_regular_20px = utils.get_font('Amazon_Ember_Rg', 20)
        self.amazon_ember_regular_16px = utils.get_font('Amazon_Ember_Rg', 16)
        self.amazon_ember_heavy_30px = utils.get_font('Amazon_Ember_Bd', 30)
        self.amazon_ember_light_18px = utils.get_font('Amazon_Ember_Lt', 18)
        self.amazon_ember_light_20px = utils.get_font('Amazon_Ember_Lt', 20)
        self.amazon_ember_light_italic_20px = utils.get_font('Amazon_Ember_LtIt', 20)

        self.is_racing = True
        self.is_league_leaderboard = True
        self.leaderboard_name = "Test"
        self._total_laps = 1

        # Gradient overlay image
        gradient_img_path = TrackAssetsIconographicPngs.OBSTACLE_OVERLAY_PNG_LEAGUE_LEADERBOARD.value \
            if self.is_league_leaderboard else TrackAssetsIconographicPngs.OBSTACLE_OVERLAY_PNG.value
        self.gradient_img = utils.get_image(gradient_img_path, IconographicImageSize.FULL_IMAGE_SIZE.value)
        self.gradient_img = cv2.cvtColor(self.gradient_img, cv2.COLOR_RGBA2BGRA)
        self.gradient_alpha_rgb_mul, self.one_minus_gradient_alpha = utils.get_gradient_values(self.gradient_img)        

    def _edit_major_cv_image(self, major_cv_image, imu_info: Imu):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
            imu_info (Imu): Inertial Measurement Unit
        Returns:
            Image: Edited main camera image
        """

        major_cv_image = cv2.resize(major_cv_image, tuple(Mp4Parameter.FRAME_SIZE.value), interpolation = cv2.INTER_AREA)

        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(major_cv_image, self.gradient_alpha_rgb_mul,
                                              self.one_minus_gradient_alpha)

        # Top left location of the picture
        loc_x, loc_y = XYPixelLoc.SINGLE_AGENT_DISPLAY_NAME_LOC.value

        # Display name (Racer name/Model name)
        display_name = self.racecar_name
        display_name_txt = display_name if len(display_name) < 15 else "{}...".format(display_name[:15])
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=display_name_txt,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_regular_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Lap Counter
        loc_y += 30
        # total_evaluation_time (Race time)
        if self.start_time == 0:
            self.start_time = Time.from_msg(imu_info.header.stamp).nanoseconds / 1e6
        
        total_eval_milli_seconds = Time.from_msg(imu_info.header.stamp).nanoseconds / 1e6 - self.start_time
        time_delta = datetime.timedelta(milliseconds=total_eval_milli_seconds)
        total_eval_time_text = "Time | {}".format(utils.milliseconds_to_timeformat(time_delta))
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=total_eval_time_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_18px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Speed
        loc_x, loc_y = XYPixelLoc.SPEED_EVAL_LOC.value
        if self.is_league_leaderboard:
            loc_x, loc_y = XYPixelLoc.SPEED_LEADERBOARD_LOC.value

        #speed_text = "{} m/s".format(utils.get_speed_formatted_str(mp4_video_metrics_info[self.racecar_index].throttle))
        speed_text = "{:.01f} m/s²".format(imu_info.linear_acceleration.x)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=speed_text,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)
        # Leaderboard name
        if self.is_league_leaderboard:
            loc_x, loc_y = XYPixelLoc.LEADERBOARD_NAME_LOC.value
            major_cv_image = utils.write_text_on_image(image=major_cv_image, text=self.leaderboard_name,
                                                       loc=(loc_x, loc_y), font=self.amazon_ember_regular_16px,
                                                       font_color=RaceCarColorToRGB.White.value,
                                                       font_shadow_color=RaceCarColorToRGB.Black.value)
        # Evaluation type
        loc_x, loc_y = XYPixelLoc.RACE_TYPE_EVAL_LOC.value
        if self.is_league_leaderboard:
            loc_x, loc_y = XYPixelLoc.RACE_TYPE_RACE_LOC.value
        race_text = "race" if self.is_racing else "evaluation"
        evaluation_type_txt = "{} {}".format(RACE_TYPE_TO_VIDEO_TEXT_MAPPING[self.race_type], race_text)
        major_cv_image = utils.write_text_on_image(image=major_cv_image, text=evaluation_type_txt,
                                                   loc=(loc_x, loc_y), font=self.amazon_ember_light_italic_20px,
                                                   font_color=RaceCarColorToRGB.White.value,
                                                   font_shadow_color=RaceCarColorToRGB.Black.value)

        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)
        return major_cv_image

    def edit_image(self, major_cv_image, imu_data):
        major_cv_image = self._edit_major_cv_image(major_cv_image, imu_data)
        return cv2.cvtColor(major_cv_image, cv2.COLOR_BGRA2RGB)
