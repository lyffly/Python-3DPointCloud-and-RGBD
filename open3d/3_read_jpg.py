# coding = utf-8
# coding by liuyunfei
# origin code from open3d samples(github)

import numpy as np
import open3d as op3


if __name__ == "__main__":
    # 读图片文件jpg
    img = op3.io.read_image('demodata/lena_color.jpg')
    print(img)

    # 保存图片
    op3.io.write_image("demodata/demo_jpg.jpg",img)





