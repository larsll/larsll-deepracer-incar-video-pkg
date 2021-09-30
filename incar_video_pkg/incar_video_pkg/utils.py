""" util module for all the mp4 saving
"""
import errno
import os
import logging
import numpy as np
import cv2
import rospkg
import threading
from PIL import ImageFont, ImageDraw, Image

from incar_video_pkg.logger import Logger

LOG = Logger(__name__, logging.INFO).get_logger()

CUSTOM_FILES_PATH = "./output"

IMAGE_CACHE = dict()

def get_font(font_name, font_size):
    """Helper method that returns an ImageFont object for the desired font if
       available otherwise returns default font

    Args:
        font_name (str): String of the desired font
        font_size (str): Size of the font in points

    Returns:
        ImageFont: ImageFont object with the given font name
    """
    try:
        font_dir = "/opt/aws/deepracer/lib/device_console/static/bootstrap/3.3.7/AmazonEmber/"
        font_path = os.path.join(font_dir, font_name + '.ttf')
        font = ImageFont.truetype(font_path, font_size)
    except (OSError, IOError):
        LOG.info("%s unsupported font, using default font", font_name)
        font = ImageFont.load_default()
    return font

def get_image(icon_name, img_size=None, is_rgb=False):
    """ Given the icon_name in the track_iconography folder without png, gives back cv2 image
    with all 4 channels
    Args:
        icon_name (str): The name of the icon in the track_iconography folder
        img_size (tuple): If you want to resize the image (width, height) (default: {None})
    Returns:
        Image: The cv2 image read from the .png file
    """
    global IMAGE_CACHE
    if icon_name in IMAGE_CACHE:
        return IMAGE_CACHE[icon_name]
    try:
        track_iconography_dir = '/opt/aws-addon/images/'
        image_path = os.path.join(track_iconography_dir, icon_name + '.png')
        image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
        if img_size:
            image = cv2.resize(image, img_size)
        IMAGE_CACHE[icon_name] = image
        return image
    except (OSError, IOError, Exception) as err_msg:
        log_and_exit("Iconography image does not exists or corrupt image: {}".format(err_msg))

def draw_shadow(draw_obj, text, font, x_loc, y_loc, shadowcolor, anchor):
    """Helper method that draws a shadow around given text for a given ImageDraw

    Args:
        draw_obj (ImageDraw): ImageDraw object where the shadow will drawn
        text (str): String to draw the shadow around
        font (ImageFont): The font of the string that the shadow will be drawn around
        x_loc (int): X location of the text string
        y_loc (int): Y location of text string
        shadowcolor (str): Color of the shadow
    """
    draw_obj.text((x_loc - 1, y_loc - 1), text, font=font, fill=shadowcolor, anchor=anchor)
    draw_obj.text((x_loc + 1, y_loc - 1), text, font=font, fill=shadowcolor, anchor=anchor)
    draw_obj.text((x_loc - 1, y_loc + 1), text, font=font, fill=shadowcolor, anchor=anchor)
    draw_obj.text((x_loc + 1, y_loc + 1), text, font=font, fill=shadowcolor, anchor=anchor)

def write_text_on_image(image, text, loc, font, font_color, font_shadow_color, anchor="la"):
    """This function is used to write the text on the image using cv2 writer

    Args:
        image (Image): The image where the text should be written
        text (str): The actual text data to be written on the image
        loc (tuple): Pixel location (x, y) where the text has to be written
        font (ImageFont): The font style object
        font_color (tuple): RGB value of the font
        font_shadow_color (tuple): RGB color of the font shawdow

    Returns:
        Image: Edited image
    """
    pil_im = Image.fromarray(image)
    draw = ImageDraw.Draw(pil_im)
    draw_shadow(draw, text, font, loc[0], loc[1], font_shadow_color, anchor)
    draw.text(loc, text, font=font, fill=font_color, anchor=anchor)
    return np.array(pil_im)

def create_folder_path(camera_dir_list):
    """ Create directory if the folder path does not exist
    Arguments:
        camera_dir_list (list): List of folder paths
    """
    for path in camera_dir_list:
        dir_path = os.path.dirname(path)
        # addressing mkdir and check directory race condition:
        # https://stackoverflow.com/questions/12468022/python-fileexists-error-when-making-directory/30174982#30174982
        # TODO: change this to os.makedirs(simtrace_dirname, exist_ok=True) when we migrate off python 2.7
        try:
            os.makedirs(dir_path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
            LOG.error("File already exist %s", dir_path)

def milliseconds_to_timeformat(dtime_delta):
    """ Convert milliseconds to mm:ss.000 format
    Args:
        dt (datetime.timedelta): Datetime delta value
    Returns:
        (str): String in 00:00.000 time format (mm:ss.millisecond)
    """
    _, dt_hr_rem = divmod(dtime_delta.seconds, 3600)
    dt_min, dt_sec = divmod(dt_hr_rem, 60)
    dt_milli_sec = dtime_delta.microseconds // 1000
    return "{:02d}:{:02d}.{:03d}".format(dt_min, dt_sec, dt_milli_sec)

def get_speed_formatted_str(speed):
    """ Returns the speed with always two whole numbers and two decimal value.
    Example: 03.45
    Args:
        speed (float): The actual speed of the car
    Returns:
        str: The text format of the speed
    """
    speed_str = "{:0.2f}".format(round(speed, 2))
    return speed_str.zfill(5)


def resize_image(image, scale_ratio):
    """ Resize the image to a given scale
    Args:
        image (Image): Image that should be rescaled
        scale_ratio: The scale ratio of the image.
    Returns:
        Image: resized image as per the scale
    """
    rows, cols, _ = image.shape
    return cv2.resize(image, (int(cols//scale_ratio), int(rows//scale_ratio)))

def get_resized_alpha(image, scale_ratio):
    """ Resize the image to the scale specified and get alpha value
    Args:
        image (Image): Image that has to resized
    Returns:
        Numpy.Array: alpha value of the image
    """
    resized_img = resize_image(image, scale_ratio)
    # The 4th channel is the alpha channel. Normalize alpha channels from 0-255 to 0-1
    return resized_img[:, :, 3] / 255.0


def get_gradient_values(gradient_img, multiplier=1):
    """ Given the image gradient returns gradient_alpha_rgb_mul and one_minus_gradient_alpha.
    These pre-calculated numbers are used to apply the gradient on the camera image

    Arguments:
        gradient_img (Image): Gradient image that has to applied on the camera image
        multiplier (float): This decides what percentage of gradient images alpha has to be applied.
                            This is useful in fading feature.

    Returns:
        (tuple): gradient_alpha_rgb_mul (Numpy.Array) gradient_img * gradient_alpha value
                 one_minus_gradient_alpha (Numpy.Array) (1 - gradient_alpha)
    """
    (height, width, _) = gradient_img.shape
    gradient_alpha = (gradient_img[:, :, 3] / 255.0 * multiplier).reshape(height, width, 1)

    gradient_alpha_rgb_mul = gradient_img * gradient_alpha
    one_minus_gradient_alpha = (1 - gradient_alpha).reshape(height, width)
    return gradient_alpha_rgb_mul, one_minus_gradient_alpha

def apply_gradient(main_image, gradient_alpha_rgb_mul, one_minus_gradient_alpha):
    """ The gradient on the image is overlayed so that text looks visible and clear.
    This leaves a good effect on the image.
    The older code took 6.348s for 1000 runs

    Numpy broadcasting is slower than normal python
    major_cv_image_1[:, :, :4] = (gradient_alpha_rgb_mul + (major_cv_image_1 * one_minus_gradient_alpha))[:, :, :4]
    Timeit 1000 runs - 6.523s

    The current code takes - 5.131s for 1000 runs

    Args:
        main_image (Image): The main image where gradient has to be applied
        gradient_alpha_rgb_mul (Numpy.Array): gradient_img * gradient_alpha value
        one_minus_gradient_alpha (Numpy.Array): (1 - gradient_alpha)
    Returns:
        Image: Gradient applied image
    """
    for channel in range(0, 4):
        main_image[:, :, channel] = gradient_alpha_rgb_mul[:, :, channel] + \
            (main_image[:, :, channel] * one_minus_gradient_alpha)
    return main_image


class DoubleBuffer(object):
    def __init__(self, clear_data_on_get=True):
        self.read_buffer = None
        self.write_buffer = None
        self.clear_data_on_get = clear_data_on_get
        self.cv = threading.Condition()

    def clear(self):
        with self.cv:
            self.read_buffer = None
            self.write_buffer = None

    def put(self, data):
        with self.cv:
            self.write_buffer = data
            self.write_buffer, self.read_buffer = self.read_buffer, self.write_buffer
            self.cv.notify()

    def get(self, block=True, timeout=None):
        """
        Get data from double buffer

        Args:
            block (bool): True for waiting when read buffer is None, False for raising DoubleBuffer.Empty
            timeout (None/float): Wait timeout for condition wait. If None is past in, it will wait
            forever until cv.notify is called

        Returns:
            Any: anything from double buffer
        """
        with self.cv:
            if not block:
                if self.read_buffer is None:
                    raise DoubleBuffer.Empty
            else:
                if self.read_buffer is None:
                    # The return value is True unless a given timeout expired, in which case it is False
                    # Changed in version 3.2: Previously, the method always returned None.
                    self.cv.wait(timeout=timeout)
                    if self.read_buffer is None:
                        raise DoubleBuffer.Empty()
            data = self.read_buffer
            if self.clear_data_on_get:
                self.read_buffer = None
            return data

    def get_nowait(self):
        return self.get(block=False)

    class Empty(Exception):
        pass

def force_list(val):
    if type(val) is not list:
        val = [val]
    return val

def log_and_exit(str):
    LOG.error(str)
    pass