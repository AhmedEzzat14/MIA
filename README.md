# Shape and Color Detection Using Classical Methods

---

## 1. **Project Overview**
This project implements **shape and color detection** using **classical computer vision techniques** in OpenCV. The goal of the task is to identify basic geometric shapes (such as triangles, squares, rectangles, and circles) and detect specific colors (in this case, yellow) in an image without relying on modern deep learning models.

---

## 2. **Approach Using Classical Methods**
### 2.1. **Image Preprocessing**
- **Color and Grayscale Conversion**: 
  - The input image is first loaded using `cv2.imread()`.
  - It is converted to grayscale for shape detection using edge detection.
  - For color detection, the image is converted to **HSV (Hue, Saturation, Value)** color space, which is more suitable for distinguishing colors compared to the RGB format.

### 2.2. **Color Detection**
- A mask is created to detect the color **yellow** using a predefined range of HSV values. The `cv2.inRange()` function is used to isolate the pixels that fall within the yellow color range.
- **Canny Edge Detection** (`cv2.Canny()`):
  - Edges are detected in both the grayscale image and the yellow color mask.
  - The edges from both sources are combined using `cv2.bitwise_or()` to create a unified edge map for further shape detection.

### 2.3. **Shape Detection**
- **Contour Detection**:
  - The edges are processed using `cv2.findContours()` to detect the boundaries of different shapes in the image.
  
- **Shape Classification**:
  - A custom function `classify_shape` is used to classify the detected contours into basic shapes:
    - **Triangle**: If the contour has 3 vertices.
    - **Square** or **Rectangle**: If the contour has 4 vertices, the aspect ratio of the bounding box is used to distinguish between squares (aspect ratio close to 1) and rectangles (other aspect ratios).
    - **Circle**: If the contour’s circularity (based on area and perimeter) exceeds a threshold value.
    - **Unknown**: If the shape doesn't match any of the above categories.

---

## 3. **Algorithms and Techniques Used**
### 3.1. **Edge Detection**:
- **Canny Edge Detection** is employed to detect edges in the image. This is crucial for both color-based and shape-based detection, as contours are extracted from the edge maps.

### 3.2. **Color Detection with HSV**:
- The HSV color space is used for **color segmentation**. By defining a range of yellow hues, a binary mask is created where the detected color is isolated. This method allows for detecting specific colors without the complexity of modern deep learning models.

### 3.3. **Contour Analysis for Shape Detection**:
- **Contours** are extracted using `cv2.findContours()`, and shape classification is performed based on:
  - The **number of vertices** in the contour approximation (`cv2.approxPolyDP()`).
  - The **aspect ratio** of the bounding box for quadrilateral shapes.
  - The **circularity** metric for distinguishing circles from other round shapes.

---

## 4. **Challenges and Insights**
### 4.1. **Accuracy of Classical Techniques**:
- **Shape approximation** using `cv2.approxPolyDP()` can sometimes lead to inaccurate results, especially with irregular or noisy contours. Without the robustness of deep learning models, the classification of shapes heavily depends on the quality of the contours.

### 4.2. **Color Detection with Fixed Thresholds**:
- Using fixed HSV thresholds for color detection is less flexible than deep learning approaches that learn from data. Variations in lighting conditions or different shades of the target color can significantly affect the performance of the color detection algorithm. Deep learning methods typically handle these variations more effectively by learning invariant features.

### 4.3. **Edge Cases**:
- **Overlapping shapes** or **distorted objects** can lead to misclassification due to limitations in contour detection and approximation.
- **Complex backgrounds** may introduce noise into the edge detection process, making it difficult to isolate the relevant shapes.

### 4.4. **Insights**:
- Classical methods are computationally efficient and can work well for **simple tasks** such as detecting basic shapes and colors. However, they lack the generalization power of deep learning methods, especially when dealing with noisy or complex images.
- While classical methods are interpretable and easier to debug, modern tools like CNNs (Convolutional Neural Networks) provide much higher accuracy and robustness, especially in real-world applications where conditions are unpredictable.

---

## 5. **Future Improvements**
- **Adaptive Color Thresholding**: To make color detection more robust to varying lighting conditions, an adaptive method for determining color thresholds could be implemented.
- **Shape Recognition with Deep Learning**: Modern deep learning techniques like **Convolutional Neural Networks (CNNs)** could be integrated to improve shape detection accuracy, especially for complex or overlapping shapes.
- **Incorporating Noise Reduction**: Preprocessing techniques such as **Gaussian blur** or **median filtering** could be added to reduce noise before edge detection, improving the quality of the contours.

---

### Final Notes
This project demonstrates that classical computer vision techniques can be highly effective for certain tasks such as shape and color detection. However, these methods have limitations in complex scenarios, where deep learning approaches would provide superior performance and robustness.
