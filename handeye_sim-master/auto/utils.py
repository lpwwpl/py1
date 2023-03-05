# _*_ coding:utf-8 _*_
# @time: 2020/9/28 下午4:06
# @author: 张新新
# @email: 1262981714@qq.com

import numpy as np
import transforms3d
import json


def dict_tranform_to_save_term(data):
    result_dict = {}
    if type(data) is list:
        result_dict['type'] = 'list'
        result_dict['data'] = []
        for temp_data in data:
            result_dict['data'].append(dict_tranform_to_save_term(temp_data))
        return result_dict
    elif type(data) is str:
        result_dict['type'] = 'str'
        result_dict['data'] = data
        return result_dict
    elif type(data) is int:
        result_dict['type'] = 'int'
        result_dict['data'] = data
        return result_dict
    elif type(data).__name__ == 'float' :
        result_dict['type'] = 'float'
        result_dict['data'] = data
        return result_dict
    elif type(data).__name__ == 'float32' or type(data).__name__ == 'float64':
        result_dict['type'] = 'float'
        result_dict['data'] = data.astype(float)
        return result_dict
    elif type(data).__name__=='dict':
        result_dict['type'] = 'dict'
        for key in data:
            result_dict[key] = dict_tranform_to_save_term(data[key])
        return result_dict
    elif type(data) is np.ndarray:
        result_dict['type'] = 'ndarray'
        shape = []
        data_shape = data.shape
        for i in range(len(data.shape)):
            shape.append(data_shape[i])
        # result_dict['shape'] = shape
        result_dict['data'] = data.tolist()
        return result_dict
def dict_tranform_to_norm_term(data):
    if data['type'] == 'list':
        result = []
        for temp_data in data['data']:
            result.append(dict_tranform_to_norm_term(temp_data))
        return result
    elif data['type'] == 'str':
        result = data['data']
        return result
    elif data['type'] == 'int':
        result = data['data']
        return result
    elif data['type'] == 'float':
        result = data['data']
        return result
    elif data['type'] == 'dict':
        result = {}
        for key in data:
            if key =='type':
                continue
            result[key] = dict_tranform_to_norm_term(data[key])
        return result
    elif data['type'] == 'ndarray':
        result = np.array(data['data'])
        return result







def json_save(data,file):
    with open(file, 'w') as f_obj:
        json.dump(dict_tranform_to_save_term(data), f_obj)

def json_load(file):
    with open(file,'r') as f:
        data = json.load(f)
    return dict_tranform_to_norm_term(data)











# def json_load():
if __name__ == '__main__':
    dict = {
        'a':0,
        'b':1.1,
        'c':[1,2,3],
        'd':{
            "d1":1
        },
        'e':np.array([0,0,0])
    }
    print(type(dict))
    dict2 = dict_tranform_to_save_term(dict)
    dict1 = dict_tranform_to_norm_term(dict2)
    print(dict1)

