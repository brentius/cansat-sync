"""GPL-3.0-only. Raw grey image to the calibrated pinhole image used by TRN.

Five-coefficient Brown-Conrady model (k1,k2,p1,p2,k3), in the convention
documented by OpenCV calib3d: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
No OpenCV dependency. Input/output share camera dimensions and intrinsics;
calibration must refer to the actual ISP crop and output resolution. A zero
model means the input is already pinhole/rectified. No coefficients are inferred.
Two float32 maps cost 8 bytes/pixel; interpolation uses bounded row strips.
This implements correction, not calibration or validation of a real lens model.
"""
import numpy as np


class Rectifier:
    def __init__(self, camera, coefficients=(0., 0., 0., 0., 0.)):
        self.camera = camera
        values = np.asarray(coefficients, dtype=float)
        if values.shape != (5,) or not np.isfinite(values).all():
            raise ValueError('five finite distortion coefficients required')
        self.identity = not np.any(values)
        self.maps = None
        if self.identity:
            return
        k1, k2, p1, p2, k3 = values
        x = (np.arange(camera.width, dtype=np.float32)[None, :] - camera.cx)/camera.fx
        y = (np.arange(camera.height, dtype=np.float32)[:, None] - camera.cy)/camera.fy
        r2 = x*x + y*y
        radial = 1 + r2*(k1 + r2*(k2 + r2*k3))
        mx = camera.fx*(x*radial + 2*p1*x*y + p2*(r2 + 2*x*x)) + camera.cx
        my = camera.fy*(y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y) + camera.cy
        if not np.isfinite(mx).all() or not np.isfinite(my).all():
            raise ValueError('distortion model produces nonfinite mapping')
        self.maps = mx.astype(np.float32), my.astype(np.float32)

    def apply(self, image):
        c = self.camera
        if image.shape != (c.height, c.width) or image.dtype not in (np.uint8, np.uint16):
            raise ValueError('raw image must match calibrated grey dimensions/type')
        if self.identity:
            return image
        output = np.zeros_like(image)
        for row in range(0, c.height, 32):
            mx, my = (m[row:row+32] for m in self.maps)
            valid = (mx >= 0) & (my >= 0) & (mx <= c.width-1) & (my <= c.height-1)
            # Clip before integer conversion, including finite extreme models.
            x = np.clip(mx, 0, c.width-1)
            y = np.clip(my, 0, c.height-1)
            x0, y0 = x.astype(np.intp), y.astype(np.intp)
            x1, y1 = np.minimum(x0+1, c.width-1), np.minimum(y0+1, c.height-1)
            dx, dy = x-x0, y-y0
            values = ((1-dy)*((1-dx)*image[y0, x0] + dx*image[y0, x1])
                      + dy*((1-dx)*image[y1, x0] + dx*image[y1, x1]))
            output[row:row+32] = np.where(valid, np.rint(values), 0).astype(image.dtype)
        return output
