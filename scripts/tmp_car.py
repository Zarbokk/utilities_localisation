from scipy.spatial.transform import Rotation as R

import numpy as np
r=np.asarray([[-0.0031492212495974376, -0.34023754131384776, 0.9403342479598564],[ -0.9999925768464749,
    -0.0010161312944974607, -0.003716682544870674],[ 0.002220057987550638, -0.9403389723700173,
    -0.34023181565607885]])
print(r.shape)
r= R.from_dcm(r)
print(r.as_euler('xyz', degrees=True))

