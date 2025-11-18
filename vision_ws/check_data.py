import glob
import os
from PIL import Image
from tqdm import tqdm

# 데이터셋 경로 (본인 경로에 맞게 수정 필요하면 수정하세요)
# 로그를 보니 아래 경로에 이미지가 있을 것으로 추정됩니다.
dataset_path = '/home/hk/aicar_project/vision_ws' 

def check_images(directory):
    print(f"Checking images in {directory}...")
    # jpg, png, jpeg 등 모든 이미지 파일 검색
    image_files = glob.glob(os.path.join(directory, '**', '*.*'), recursive=True)
    image_files = [f for f in image_files if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))]
    
    corrupt_files = []
    
    for file_path in tqdm(image_files):
        try:
            # 이미지를 열어서 검증 시도
            with Image.open(file_path) as img:
                img.verify() 
        except Exception as e:
            print(f"\n[Corrupt Found] {file_path}: {e}")
            corrupt_files.append(file_path)
            
    return corrupt_files

if __name__ == "__main__":
    # train, valid 폴더 검사
    train_dir = os.path.join(dataset_path, 'train')
    valid_dir = os.path.join(dataset_path, 'valid') # 폴더명이 val인지 valid인지 확인하세요
    
    bad_files = check_images(train_dir) + check_images(valid_dir)
    
    if bad_files:
        print("\n============== 발견된 손상 파일 ==============")
        for f in bad_files:
            print(f)
        print("==============================================")
        print("위 파일들을 삭제하거나 정상적인 파일로 교체해주세요.")
    else:
        print("\n모든 이미지가 정상입니다.")
