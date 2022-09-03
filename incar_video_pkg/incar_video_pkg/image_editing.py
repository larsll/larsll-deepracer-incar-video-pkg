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
from incar_video_pkg.constants import (ColorMap, TrackAssetsIconographicPngs,
                                       XYPixelLoc, Mp4Parameter)
from incar_video_pkg.image_editing_interface import ImageEditingInterface

LOG = Logger(__name__, logging.INFO).get_logger()


class ImageEditing(ImageEditingInterface):
    """ Image editing class for head to bot, time-trail, obstacle where
    there is only single agent
    """

    def __init__(self, racecar_name):
        """ Initializing the required data for video frame.

        Arguments:
            racecar_name (str): Display name of car
        """

        self.racecar_name = racecar_name
        self.start_time = 0

        # Store the font which we will use to write the phase with
        self.amazon_ember_regular_20px = utils.get_font('Amazon_Ember_Rg', 20)
        self.amazon_ember_regular_18px = utils.get_font('Amazon_Ember_Rg', 18)
        self.amazon_ember_regular_16px = utils.get_font('Amazon_Ember_Rg', 16)

        self.amazon_ember_heavy_30px = utils.get_font('Amazon_Ember_Bd', 30)
        self.amazon_ember_light_16px = utils.get_font('Amazon_Ember_Lt', 16)
        self.amazon_ember_light_18px = utils.get_font('Amazon_Ember_Lt', 18)
        self.amazon_ember_light_20px = utils.get_font('Amazon_Ember_Lt', 20)
        self.amazon_ember_light_italic_20px = utils.get_font(
            'Amazon_Ember_LtIt',
            20)

        # Gradient overlay image
        gradient_img_path = TrackAssetsIconographicPngs.VIDEO_OVERLAY_PNG.value
        self.gradient_img = utils.get_image(
            gradient_img_path, Mp4Parameter.FRAME_SIZE.value)
        self.gradient_img = cv2.cvtColor(
            self.gradient_img, cv2.COLOR_RGBA2BGRA)

        # Top left location of the picture
        loc_x, loc_y = XYPixelLoc.SINGLE_AGENT_DISPLAY_NAME_LOC.value

        # Display name (Racer name/Model name)
        display_name = self.racecar_name
        display_name_txt = display_name if len(
            display_name) < 15 else "{}...".format(display_name[:15])
        self.gradient_img = utils.write_text_on_image(
            image=self.gradient_img, text=display_name_txt, loc=(loc_x, loc_y),
            font=self.amazon_ember_regular_20px,
            font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value)

        # Acceleration label
        loc_x, loc_y = XYPixelLoc.ACCEL_LBL_LOC.value
        accel_lbl_text = "Accel | m/s²"
        self.gradient_img = utils.write_text_on_image(
            image=self.gradient_img, text=accel_lbl_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value)

        # Gyro label
        loc_x, loc_y = XYPixelLoc.ANGULAR_LBL_LOC.value
        gyro_lbl_text = "Gyro | rad/s "
        self.gradient_img = utils.write_text_on_image(
            image=self.gradient_img, text=gyro_lbl_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value)

        self.gradient_alpha_rgb_mul, self.one_minus_gradient_alpha = \
            utils.get_gradient_values(self.gradient_img)

    def _edit_major_cv_image(self, major_cv_image, imu_info: Imu):
        """ Apply all the editing for the Major 45degree camera image
        Args:
            major_cv_image (Image): Image straight from the camera
            imu_info (Imu): Inertial Measurement Unit
        Returns:
            Image: Edited main camera image
        """

        major_cv_image = cv2.resize(
            major_cv_image, tuple(Mp4Parameter.FRAME_SIZE.value),
            interpolation=cv2.INTER_AREA)

        # Applying gradient to whole major image and then writing text
        major_cv_image = utils.apply_gradient(
            major_cv_image, self.gradient_alpha_rgb_mul,
            self.one_minus_gradient_alpha)

        # total_evaluation_time (Race time)
        loc_x, loc_y = XYPixelLoc.TIME_LOC.value
        if self.start_time == 0:
            self.start_time = Time.from_msg(
                imu_info.header.stamp).nanoseconds / 1e6

        total_eval_milli_seconds = Time.from_msg(
            imu_info.header.stamp).nanoseconds / 1e6 - self.start_time
        time_delta = datetime.timedelta(milliseconds=total_eval_milli_seconds)
        total_eval_time_text = "{}".format(
            utils.milliseconds_to_timeformat(time_delta))
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=total_eval_time_text, loc=(loc_x,
                                                                  loc_y),
            font=self.amazon_ember_light_18px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value)

        # Acceleration
        loc_x, loc_y = XYPixelLoc.ACCEL_X_LOC.value
        accel_x_text = "{:+.1f}".format(imu_info.linear_acceleration.x)
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=accel_x_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value, anchor="ra")

        loc_x, loc_y = XYPixelLoc.ACCEL_Y_LOC.value
        accel_y_text = "{:+.1f}".format(imu_info.linear_acceleration.y)
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=accel_y_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value, anchor="ra")

        loc_x, loc_y = XYPixelLoc.ACCEL_Z_LOC.value
        accel_z_text = "{:+.1f}".format(imu_info.linear_acceleration.z)
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=accel_z_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value, anchor="ra")

        # Gyro
        loc_x, loc_y = XYPixelLoc.ANGULAR_X_LOC.value
        angular_x_text = "{:+.1f}".format(imu_info.angular_velocity.x)
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=angular_x_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value, anchor="ra")

        loc_x, loc_y = XYPixelLoc.ANGULAR_Y_LOC.value
        angular_y_text = "{:+.1f}".format(imu_info.angular_velocity.y)
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=angular_y_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value, anchor="ra")

        loc_x, loc_y = XYPixelLoc.ANGULAR_Z_LOC.value
        angular_z_text = "{:+.1f}".format(imu_info.angular_velocity.z)
        major_cv_image = utils.write_text_on_image(
            image=major_cv_image, text=angular_z_text, loc=(loc_x, loc_y),
            font=self.amazon_ember_light_16px, font_color=ColorMap.White.value,
            font_shadow_color=ColorMap.Black.value, anchor="ra")

        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2BGRA)
        return major_cv_image

    def edit_image(self, major_cv_image, imu_data):
        major_cv_image = self._edit_major_cv_image(major_cv_image, imu_data)
        return cv2.cvtColor(major_cv_image, cv2.COLOR_BGRA2RGB)
