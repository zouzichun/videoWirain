
import os

def list_jpg_to_txt(output_file='dataset.txt'):
    """列出当前目录所有jpg文件并写入txt"""
    jpg_files = [f for f in os.listdir('./dataset') 
                if f.lower().endswith('.jpg')]
    
    with open(output_file, 'w') as f:
        for file_name in jpg_files:
            f.write('./dataset/' + file_name + '\n')
    
    print(f"共找到 {len(jpg_files)} 个JPG文件，已保存到 {output_file}")

if __name__ == '__main__':
    list_jpg_to_txt()
