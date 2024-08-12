import yaml

with open('calibration/cam1.yaml','r') as f:
    data=yaml.safe_load(f)



def cam_mats_to_params(data):
    
    cam_mats=data['camera_matrix']['data']
    fx=cam_mats[0]
    cx=cam_mats[2]
    fy=cam_mats[4]
    cy=cam_mats[5]

    return [fx,fy,cx,cy]

print(cam_mats_to_params(data))