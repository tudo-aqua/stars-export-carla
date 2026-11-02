from enum import Enum

import cv2

class DownscalingMethod(Enum):
    INTER_NEAREST = cv2.INTER_NEAREST	      #Speed: Fast,     Quality: Low	    Notes: Pixelated results; very fast.
    INTER_LINEAR = cv2.INTER_LINEAR	          #Speed: Medium,   Quality: Medium	    Notes: Default; good for most cases.
    INTER_AREA=cv2.INTER_AREA	              #Speed: Slow,     Quality: High	    Notes: Best for downscaling; avoids aliasing.
    INTER_CUBIC	=cv2.INTER_CUBIC              #Speed: Slower,   Quality: High	    Notes: Smooth and high-quality results.
    INTER_LANCZOS4=cv2.INTER_LANCZOS4	      #Speed: Slowest,  Quality: Very High	Notes: Best quality; computationally expensive.
    INTER_LINEAR_EXACT=cv2.INTER_LINEAR_EXACT #Speed: Medium,   Quality: Medium	    Notes: Precise bilinear interpolation.