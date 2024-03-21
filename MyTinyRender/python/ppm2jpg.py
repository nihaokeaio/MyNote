from PIL import Image  
import sys
  
def ppm_to_jpg(ppm_path, jpg_path):  
    # 打开PPM图片
    print(ppm_path)  
    print(jpg_path)
    with Image.open(ppm_path) as ppm_image:
        # 将图片保存为JPG格式  
        ppm_image.save(jpg_path, "JPEG")
    img=Image.open(jpg_path)
    img.show();
  
if __name__ == "__main__": 
    nums=len(sys.argv)
    print(sys.argv)
    ppm_file = sys.argv[-2]  # 输入的PPM图片路径  
    jpg_file = sys.argv[-1]  # 输出的JPG图片路径  
    ppm_to_jpg(ppm_file, jpg_file)  
    print(f"Conversion complete! {ppm_file} has been converted to {jpg_file}")
