Retrieved from: https://openaccess.thecvf.com/content/CVPR2022/papers/Feng_Rethinking_Efficient_Lane_Detection_via_Curve_Modeling_CVPR_2022_paper.pdf

# Notes
## Introduction
- Lane detection methods have previously relied on expensive sensors like LIDAR.
- Many recent works are proposed to detect lines using RGB.
- Three methods for lane detection methods: Segmentation, Point Detection, and Curve based.
- Segmentation and Point Detection based methods achieve state-of-the-art performance.
- Segmentation involves using texture cues to segment lane pixels and decode them into lines.
- Point detection usually adopts the R-CNN framework and detects lane lines by detecting a dense series of points on the vertical axis
- Both methods make lines using indirect proxies. Their main problem is that under occlusions or adverse weather conditions, they rely on inefficient designs too heavy for real-time tasks.
- Few methods to model lane curves.  Curve coefficients are hard to determine, and are about 8% slower.
- Bezier curves are a low-computational way of calculating curves.
- This paper aims to prove that a Bezier curve-based deep lane detector can be used to model the shape of lane lines effectively and be adverse to driving conditions.
- Also aims to implement a flip-fusion model that exploits symmetrical properties of lanes.
- Also show that Bezier curves are fast, lightweight, and accurate

### Related Work
#### Segmentation-based Lane Detection
- 