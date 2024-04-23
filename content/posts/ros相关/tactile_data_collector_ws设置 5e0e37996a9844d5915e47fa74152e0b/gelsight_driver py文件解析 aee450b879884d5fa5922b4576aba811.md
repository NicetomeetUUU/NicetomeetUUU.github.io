---
title: "gelsight_driver.py文件解析"
date: 2023-10-22
description: "在gazebo仿真下的gelsight数据采集环境"
type: "post"
tags: ["blog","ros相关"]
---

# gelsight_driver.py文件解析

## 1. 图像基础操作函数:

```python
#基础图像处理部分
def show_normalized_img(name, img):
    draw = img.copy()
    draw -= np.min(draw)
    draw = draw / np.max(draw)
    cv2.imshow(name, draw)
    return draw
#归一化图像, 变为0-1间的值

def gkern2(kernlen=21, nsig=3):
    """Returns a 2D Gaussian kernel array."""

    # create nxn zeros
    inp = np.zeros((kernlen, kernlen))
    # set element at the middle to one, a dirac delta
    inp[kernlen // 2, kernlen // 2] = 1
    # gaussian-smooth the dirac, resulting in a gaussian filter mask
    return fi.gaussian_filter(inp, nsig)
#返回一个2d高斯核函数

def gaus_noise(image, sigma):
    row, col = image.shape
    mean = 0
    gauss = np.random.normal(mean, sigma, (row, col))
    gauss = gauss.reshape(row, col)
    noisy = image + gauss
    return noisy
#为图像按sigma参数增加噪声

def derivative(mat, direction):
    assert (direction == 'x' or direction == 'y'), "The derivative direction must be 'x' or 'y'"
    kernel = None
    if direction == 'x':
        kernel = [[-1.0, 0.0, 1.0]]
    elif direction == 'y':
        kernel = [[-1.0], [0.0], [1.0]]
    kernel = np.array(kernel, dtype=np.float64)
    return cv2.filter2D(mat, -1, kernel) / 2.0
#使用一个简单的差分核（[-1, 0, 1]）来计算图像在x方向或y方向上的导数

def tangent(mat):
    dx = derivative(mat, 'x')
    dy = derivative(mat, 'y')
    img_shape = np.shape(mat)
    _1 = np.repeat([1.0], img_shape[0] * img_shape[1]).reshape(img_shape).astype(dx.dtype)
    unormalized = cv2.merge((-dx, -dy, _1))
    norms = np.linalg.norm(unormalized, axis=2)
    return (unormalized / np.repeat(norms[:, :, np.newaxis], 3, axis=2))
#计算x,y方向上的导数, 也就是坡度, 用三维向量表示(传入的mat像素点值表示亮度[或深度])

def solid_color_img(color, size):
    image = np.zeros(size + (3,), np.float64)
    image[:] = color
    return image
##创建一个指定颜色和大小的图像

def add_overlay(rgb, alpha, color):
    s = np.shape(alpha)

    opacity3 = np.repeat(alpha, 3).reshape((s[0], s[1], 3))  # * 10.0

    overlay = solid_color_img(color, s)

    foreground = opacity3 * overlay
    background = (1.0 - opacity3) * rgb.astype(np.float64)
    res = background + foreground

    res[res > 255.0] = 255.0
    res[res < 0.0] = 0.0
    res = res.astype(np.uint8)

    return res
#将一个指定颜色和透明度的覆盖层添加到RGB图像上

```

# 2. 仿真中gelsight触觉传感器类

```python
"""
	class Simulation
	@parame light_sources, background, px2m_ratio, elastomer_thickness, min_depth
					光源, 背景, 比例转换, 凝胶厚度, 最小深度(透明介质大小)
"""

class SimulationApproach:

    def __init__(self, **config):
        self.light_sources = config['light_sources']
        self.background = config['background_img']
        self.px2m_ratio = config['px2m_ratio']
        self.elastomer_thickness = config['elastomer_thickness']
        self.min_depth = config['min_depth']
				#定义光源, tactile背景图, 像素到米的比例值, 凝胶厚度与最小深度
				
        self.default_ks = 0.15
        self.default_kd = 0.5
        self.default_alpha = 5
				#参数群, 用于phong照明

        self.ka = config['ka'] or 0.8
				#环境光强度

        self.texture_sigma = config['texture_sigma'] or 0.00001
        self.t = config['t'] if 't' in config else 3
        self.sigma = config['sigma'] if 'sigma' in config else 7
        self.kernel_size = config['sigma'] if 'sigma' in config else 21

        self.max_depth = self.min_depth + self.elastomer_thickness

    def protrusion_map(self, original, not_in_touch):
		#用于将超出最大深度的像素点深度回归到最大深度
        protrusion_map = np.copy(original)
        protrusion_map[not_in_touch >= self.max_depth] = self.max_depth
        return protrusion_map

    def segments(self, depth_map):
        not_in_touch = np.copy(depth_map)
        not_in_touch[not_in_touch < self.max_depth] = 0.0
        not_in_touch[not_in_touch >= self.max_depth] = 1.0

        in_touch = 1 - not_in_touch

        return not_in_touch, in_touch

    def internal_shadow(self, elastomer_depth):
        elastomer_depth_inv = self.max_depth - elastomer_depth
        elastomer_depth_inv = np.interp(elastomer_depth_inv, (0, self.elastomer_thickness), (0.0, 1.0))
        return elastomer_depth_inv

    def apply_elastic_deformation_v1(self, protrusion_depth, not_in_touch, in_touch):
        kernel = gkern2(15, 7)
        deformation = self.max_depth - protrusion_depth

        for i in range(5):
            #     # cv2.waitKey(10)
            deformation = cv2.filter2D(deformation, -1, kernel)
        #     # show_normalized_img('deformation', deformation)
        # return deformation
        return 30 * -deformation * not_in_touch + (protrusion_depth * in_touch)

    def apply_elastic_deformation(self, protrusion_depth, not_in_touch, in_touch):
        protrusion_depth = - (protrusion_depth - self.max_depth)

        kernel = gkern2(self.kernel_size, self.sigma)
        deformation = protrusion_depth

        deformation2 = protrusion_depth
        kernel2 = gkern2(52, 9)

        for i in range(self.t):
            deformation_ = cv2.filter2D(deformation, -1, kernel)
            r = np.max(protrusion_depth) / np.max(deformation_) if np.max(deformation_) > 0 else 1
            deformation = np.maximum(r * deformation_, protrusion_depth)

            deformation2_ = cv2.filter2D(deformation2, -1, kernel2)
            r = np.max(protrusion_depth) / np.max(deformation2_) if np.max(deformation2_) > 0 else 1
            deformation2 = np.maximum(r * deformation2_, protrusion_depth)

        deformation_v1 = self.apply_elastic_deformation_v1(protrusion_depth, not_in_touch, in_touch)

        # deformation2 = protrusion_depth
        #
        for i in range(self.t):
            deformation_ = cv2.filter2D(deformation2, -1, kernel)
            r = np.max(protrusion_depth) / np.max(deformation_) if np.max(deformation_) > 0 else 1
            deformation2 = np.maximum(r * deformation_, protrusion_depth)

        #

        # for i in range(3):
        # deformation3 = protrusion_depth
        # kernel3 = gkern2(21, 7)
        # for i in range(3):
        #     deformation3_ = cv2.filter2D(deformation3, -1, kernel3)
        #     r = np.max(protrusion_depth) / np.max(deformation3_) if np.max(deformation3_) > 0 else 1
        #     deformation3 = np.maximum(r * deformation3_, protrusion_depth)

        #
        # # r = np.max(protrusion_depth) / np.max(deformation) if np.max(deformation) > 0 else 1
        # # deformation = np.maximum(r * deformation, protrusion_depth)
        # # plt.axis('off')
        #
        #
        # plt.plot(list(range(len(protrusion_depth[150]))), -1 * protrusion_depth[240], color="gray",
        #          label='Before Smoothing')
        # plt.plot(list(range(len(deformation[150]))), -1 * deformation[240], color="limegreen", linestyle='dashed',
        #          label='Single Gaussian')
        # #
        # # plt.plot(list(range(len(deformation2[150]))), -1 * deformation[150] + deformation2[150], color='red',
        # #          linestyle='dashed',
        # #          label='with ratioxxxxx')
        # # deformation_x = -1 * deformation[150] + deformation2[150] - deformation[150]
        deformation_x = 2 * deformation - deformation2
        #
        # plt.plot(list(range(len(deformation[150]))), - deformation_x[240], color="darkorange", linestyle='dashed',
        #          label='Difference of Gaussians')

        # plt.plot(list(range(len(deformation2[150]))), -deformation_v1[150],
        #          color='black',
        #          # linestyle='do',
        #          label='Previous ')

        # plt.plot(list(range(len(deformation2[150]))), deformation_x[150],
        #          color='red',
        #          linestyle='dashed',
        #          label='with ratioxxxxx')

        # tangent = lambda arr: np.array([abs(arr[i + 1] - arr[i - 1]) / 2 if i > 0 and i < len(arr) - 2 else 0 for i in
        #                        range(len(arr))])
        #
        # t = tangent(deformation2[150])
        # plt.plot(list(range(len(deformation2[240]))),
        #          deformation[150] + (np.max(deformation2[150]) / np.max(t)) * t,
        #          color='red',
        #          label='After Filtering')

        # plt.xticks([])
        # plt.yticks([])
        # plt.legend()
        # plt.show()
        # plt.clf()
        # plt.cla()

        #
        # cv2.imwrite('protrusion.png', show_normalized_img('protrusion', protrusion_depth) * 255)
        # cv2.imwrite('deformation.png', show_normalized_img('deformation', deformation) * 255)

        return self.max_depth - deformation_x

    def phong_illumination(self, T, source_dir, kd, ks, alpha):
        dot = np.dot(T, np.array(source_dir)).astype(np.float64)
        difuse_l = dot * kd
        difuse_l[difuse_l < 0] = 0.0

        dot3 = np.repeat(dot[:, :, np.newaxis], 3, axis=2)

        R = 2.0 * dot3 * T - source_dir
        V = [0.0, 0.0, 1.0]

        spec_l = np.power(np.dot(R, V), alpha) * ks

        return difuse_l + spec_l

    def generate(self, obj_depth, return_depth=False):
        # print('-----------> ', np.shape(obj_depth))
        # cv2.imwrite('object_depth.png', obj_depth)
        not_in_touch, in_touch = self.segments(obj_depth)
        protrusion_depth = self.protrusion_map(obj_depth, not_in_touch)
        elastomer_depth = self.apply_elastic_deformation(protrusion_depth, not_in_touch, in_touch)

        textured_elastomer_depth = gaus_noise(elastomer_depth, self.texture_sigma)

        out = self.ka * self.background
        out = add_overlay(out, self.internal_shadow(protrusion_depth), (0.0, 0.0, 0.0))

        T = tangent(textured_elastomer_depth / self.px2m_ratio)
        # show_normalized_img('tangent', T)
        for light in self.light_sources:
            ks = light['ks'] if 'ks' in light else self.default_ks
            kd = light['kd'] if 'kd' in light else self.default_kd
            alpha = light['alpha'] if 'alpha' in light else self.default_alpha
            out = add_overlay(out, self.phong_illumination(T, light['position'], kd, ks, alpha), light['color'])

        kernel = gkern2(3, 1)
        out = cv2.filter2D(out, -1, kernel)

        # cv2.imshow('tactile img', out)
        # cv2.imwrite('tactile_img.png', out)
        #
        if return_depth:
            return out, elastomer_depth
        return out
```