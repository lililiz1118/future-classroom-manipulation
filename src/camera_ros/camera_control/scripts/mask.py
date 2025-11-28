import cv2
import numpy as np
from PIL import Image

# 全局变量，用于存储框选的坐标和状态
drawing = False  # True 表示正在拖动
ix, iy = -1, -1  # 鼠标按下时的起始坐标
bbox = None      # 最终的边界框坐标

# 鼠标回调函数
def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, bbox

    # 左键按下时，记录起始坐标
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        print(f"鼠标按下: ({x}, {y})")

    # 鼠标移动时，如果正在拖动，则实时绘制矩形
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            img_temp = param['image'].copy()
            cv2.rectangle(img_temp, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow('Image', img_temp)

    # 左键释放时，记录结束坐标，并保存边界框
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        print(f"鼠标释放: ({x}, {y})")
        x_min = min(ix, x)
        y_min = min(iy, y)
        x_max = max(ix, x)
        y_max = max(iy, y)
        bbox = (x_min, y_min, x_max, y_max)

        cv2.rectangle(param['image'], (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        cv2.imshow('Image', param['image'])
        print(f"边界框坐标: {bbox}")

def create_mask_from_bbox(image_shape, bbox):
    """根据边界框生成黑白掩码 (0/255)"""
    mask = np.zeros(image_shape[:2], dtype=np.uint8)
    if bbox is not None:
        x_min, y_min, x_max, y_max = bbox
        mask[y_min:y_max, x_min:x_max] = 255
    return mask

# --- 主程序 ---
if __name__ == "__main__":
    image_path = '/home/jt001/realsense_images/color.png'
    try:
        img = cv2.imread(image_path)
        if img is None:
            raise FileNotFoundError("文件未找到或无法读取。")

        if img.shape[1] != 1280 or img.shape[0] != 720:
            print(f"警告：图像尺寸不是预期的 1280x720，实际尺寸为 {img.shape[1]}x{img.shape[0]}。")

        cv2.namedWindow('Image')
        cv2.setMouseCallback('Image', draw_rectangle, {'image': img})

        print("请在窗口中按住鼠标左键拖动以选择区域。")
        print("按下 'q' 键退出。")

        while True:
            cv2.imshow('Image', img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()

        # 生成并保存掩码
        if bbox:
            mask = create_mask_from_bbox(img.shape, bbox)
            output_mask_path = '/home/jt001/tracer_ws/src/camera_ros/camera_control/scripts/mask.png'

            # 转换为 PIL 1-bit 图像并保存
            pil_mask = Image.fromarray(mask).convert("1")
            pil_mask.save(output_mask_path)
            print(f"成功生成 1-bit 掩码，并保存到: {output_mask_path}")
            print("PIL 模式:", pil_mask.mode)

        else:
            print("未检测到有效边界框，未生成掩码。")

    except FileNotFoundError as e:
        print(f"错误: {e}")
        print("请确保 'input.png' 文件存在于与脚本相同的目录下。")
