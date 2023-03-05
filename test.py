#!/usr/bin/env python
#-*- coding:utf-8 -*-

"""
This file is part of QProgEdit.

QProgEdit is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

QProgEdit is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with QProgEdit.  If not, see <http://www.gnu.org/licenses/>.
"""

import os
import sys

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
# from QProgEdit import QTabManager, validate

def cursorRowChanged(index, rowFrom, rowTo):

	print(u'curorRowChanged(): %d, %d, %d' % (index, rowFrom, rowTo))

def focusLost(index):

	print(u'focusOut(): %s' % index)

def focusReceived(index):

	print(u'focusReceived(): %s' % index)

def handlerButtonClicked(index):

	print(u'handlerButtonClicked(): %s' % index)

def activateSymbolTree(treeWidgetItem):

	if hasattr(treeWidgetItem, u'activate'):
		treeWidgetItem.activate()

def runSelectedText(s):

	print('run:\n%s' % s)



import numpy as np


def split_space(x, y, z):  # 以1为粒度，分割空间
	return np.zeros((x, y, z), dtype=np.int)


def put_box(space, select_point, select_box):  # 若该放置点可以放置包裹，则将放置空间置为1
	x, y, z = select_point[0], select_point[1], select_point[2]
	d, w, h = select_box['size'][0], select_box['size'][1], select_box['size'][2]
	space[x:x + d, y:y + w, z:z + h] = 1
	return space


def judge_drawer(select_point, select_box, drawer):  # 判断新放入的包裹是否超出抽屉，最外面的顶点在抽屉内，则判定可放入
	if select_point[0] + select_box['size'][0] <= drawer['dimensions'][0] and \
			select_point[1] + select_box['size'][1] <= drawer['dimensions'][1] and \
			select_point[2] + select_box['size'][2] <= drawer['dimensions'][2]:
		return True
	else:
		return False


def judge_boxs(space, select_point, select_box):  # 判断是否与其他包裹放置空间重合,八个顶点所在位置未填充则可以放入
	x, y, z = select_point[0], select_point[1], select_point[2]
	d, w, h = select_box['size'][0], select_box['size'][1], select_box['size'][2]
	if space[x, y, z] | \
			space[x + d - 1, y, z] | space[x, y + w - 1, z] | space[x, y, z + h - 1] | \
			space[x + d - 1, y + w - 1, z] | space[x + d - 1, y, z + h - 1] | space[x, y + w - 1, z + h - 1] | \
			space[x + d - 1, y + w - 1, z + h - 1]:
		return False
	else:
		return True


def pack_box_into_drawer(list_box, drawer):  # 查看DNA（装柜顺序及姿势)的空间利用率和装柜结果
	drawer_d, drawer_w, drawer_h = drawer['dimensions']
	space = split_space(drawer_d, drawer_w, drawer_h)  # 将抽屉划分为小空间
	put_point = [[0, 0, 0]]  # 可选放置点
	drawer['boxes'] = []  # 最后装入物品的集合

	for i in range(len(list_box)):
		put_point.sort(key=lambda x: (x[2], x[1], x[0]))  # 放置点列表更新后，为保证约定的放置顺序，排序放置点
		# print(i,put_point)
		box = list_box[i]
		box_d, box_w, box_h = box['size']
		if box_d > drawer_d or box_w > drawer_w or box_h > drawer_h:  # 排除放不进空抽屉的包裹
			continue

		for index, point in enumerate(put_point):  # 依次实验在每个放置点放置包裹，如果包裹在这个位置能放成功，装入抽屉
			if judge_drawer(point, box, drawer) and judge_boxs(space, point, box):  # 如果包裹在这个位置能放进当前抽屉空间
				space = put_box(space, point, box)  # 更新空间
				drawer['boxes'].append(box)  # 装入抽屉
				# 删除放置点
				put_point.pop(index)

				# 添加新的放置点(有待改进)
				put_point.append([point[0] + box_d, point[1], point[2]])
				put_point.append([point[0], point[1] + box_w, point[2]])
				put_point.append([point[0], point[1], point[2] + box_h])
				break

	space_ratio = space.sum() / (drawer_d * drawer_w * drawer_h)
	# print('---装柜策略:', select_item)
	# print('---空间利用率:', space_ratio)
	# print('---几个没装进去', len(list_box)-len(drawer['boxes']))
	return space_ratio, drawer



import random
import math
import datetime
import numpy as np
import copy


def exchange_box(list_box):  # 随机交换两个包裹装柜顺序
	if len(list_box) != 1:
		s1, s2 = random.randint(0, len(list_box) - 1), random.randint(0, len(list_box) - 1)
		while s1 == s2:
			s2 = random.randint(0, len(list_box) - 1)
		list_box[s1], list_box[s2], = list_box[s2], list_box[s1]
	return list_box


def exchange_direction(list_box):  # 随机交换某个包裹的装柜姿势
	s = random.randint(0, len(list_box) - 1)
	box = list_box[s]
	s1, s2 = random.randint(0, len(box['size']) - 1), random.randint(0, len(box['size']) - 1)
	while s1 == s2:
		s2 = random.randint(0, len(box['size']) - 1)
	box['size'][s1], box['size'][s2], = box['size'][s2], box['size'][s1]
	list_box[s] = box
	return list_box


def crossover(list_box_f, list_box_m):  # 交叉配对（父亲，母亲）
	# 后代继承了母亲的装包裹顺序和父亲的装包裹姿势
	list_box_c = copy.deepcopy(list_box_m)
	for i in range(len(list_box_f)):
		index_max_f = list_box_f[i]['size'].index(max(list_box_f[i]['size']))
		list_box_c[i]['size'][index_max_f] = max(list_box_m[i]['size'])

		index_min_f = list_box_f[i]['size'].index(min(list_box_f[i]['size']))
		list_box_c[i]['size'][index_min_f] = min(list_box_m[i]['size'])

		index_max_m = list_box_m[i]['size'].index(max(list_box_m[i]['size']))
		index_min_m = list_box_m[i]['size'].index(min(list_box_m[i]['size']))
		index_f = list({0, 1, 2} - {index_max_f, index_min_f})[0]
		index_m = list({0, 1, 2} - {index_max_m, index_min_m})[0]
		list_box_c[i]['size'][index_f] = list_box_m[i]['size'][index_m]
	return list_box_c


def integral(list_x):
	list_integral = []
	x_sum = 0
	for x in list_x:
		x_sum += x
		list_integral.append(x_sum)
	return list_integral


def my_random(list_integral):
	p = random.uniform(0, max(list_integral))
	for i in range(len(list_integral)):
		if p < list_integral[i]:
			break
	return i


def init_ethnic(list_box, ethnic_num):  # 初始化族群，个数为ethnic_num
	list_list_box = []
	for i in range(ethnic_num):
		for j in range(100):
			if random.random() > 0.5:  # 随机交换两个包裹装柜顺序
				list_box = exchange_box(list_box)
			else:  # 随机交换某个包裹的装柜姿势
				list_box = exchange_direction(list_box)
		list_list_box.append(copy.deepcopy(list_box))
	return list_list_box


def ethnic_reproduction(list_box, ethnic_num, pair_n, variation_n, deadline, drawer):
	# 族群繁衍(初始包裹列表,族群个数,配对次数,变异次数,截止时间,抽屉)

	# 初始化族群
	list_list_box = init_ethnic(list_box, ethnic_num)
	list_value = []
	list_strategy = []
	for i in range(len(list_list_box)):
		value, strategy = pack_box_into_drawer(list_list_box[i], drawer)
		list_value.append(value)
		list_strategy.append(strategy)

	# 记录最好装抽屉策略
	value_best = max(list_value)  # 最好空间利用率
	index = list_value.index(value_best)
	strategy_best = list_strategy[index]  # 最好策略
	list_integral = integral(list_value)

	# 开始迭代
	while len(list_box) - len(strategy_best['boxes']) > 0 \
			and value_best != 1.0 \
			and datetime.datetime.now() < deadline:  # 如果有包裹装不进抽屉，当前时间大于截至时间，停止迭代，输出最优结果

		# 有放回的随机选择几对，配对繁衍后代，空间利用率越高被选中繁衍后代的概率越高
		for i in range(pair_n):
			s1, s2 = my_random(list_integral), my_random(list_integral)
			while s1 == s2:
				s2 = my_random(list_integral)
			list_box_new = crossover(list_list_box[s1], list_list_box[s2])
			list_list_box.append(list_box_new)
			value, strategy = pack_box_into_drawer(list_box_new, drawer)
			list_value.append(value)
			list_strategy.append(strategy)

		# 变异
		for i in range(len(list_list_box)):
			for j in range(variation_n):
				if random.random() > 0.5:  # 随机交换两个包裹装柜顺序
					list_list_box[i] = exchange_box(list_list_box[i])
				else:  # 随机交换某个包裹的装柜姿势
					list_list_box[i] = exchange_direction(list_list_box[i])
			value, strategy = pack_box_into_drawer(list_list_box[i], drawer)
			list_value[i] = value
			list_strategy[i] = strategy

		# 自然选择，淘汰一批DNA，控制族群规模不变
		for i in range(pair_n):
			index = list_value.index(min(list_value))
			del list_value[index]
			del list_strategy[index]
			del list_list_box[index]

		# 记录最好装抽屉策略
		value_best = max(list_value)  # 最好空间利用率
		index_best = list_value.index(value_best)
		strategy_best = list_strategy[index_best]  # 最好策略
		list_integral = integral(list_value)

	return value_best, strategy_best  # 最好空间利用率，最好装抽屉策略




def selection_strategy(list_box, list_hive, deadline, ethnic_num=20, pair_n=10, variation_n=1):
	# 待装包裹列表，柜子列表，截止时间，族群规模，配对次数，变异次数
	# 接口适配
	list_drawer = []
	for hive in list_hive:
		for i in range(len(hive['drawers'])):
			hive['drawers'][i]['hive_id'] = hive['id']
		list_drawer.extend(hive['drawers'])

	list_box_to_be_packed = list_box.copy()  # 待装包裹
	for i in range(len(list_drawer)):  # 一个抽屉一个抽屉的装
		drawer = list_drawer[i].copy()

		# 遗传算法
		value_best, strategy_best = ethnic_reproduction(list_box_to_be_packed, ethnic_num, pair_n, variation_n,
														deadline, drawer)

		# 更新待包装包裹列表，装到下一个抽屉
		list_box_tmp = []
		list_box_j_id = []
		for box_i in list_box_to_be_packed:  # 本次待装包裹
			for box_j in strategy_best['boxes']:
				list_box_j_id.append(box_j['id'])  # 本次已装包裹id
			if box_i['id'] not in list_box_j_id:
				list_box_tmp.append(box_i)
		list_box_to_be_packed = list_box_tmp  # 下次待装包裹

		# 记录
		print('最好的空间利用率', value_best)
		print('还有几个没装进去', len(list_box_to_be_packed), list_box_to_be_packed)
		print('最好的装柜策略', strategy_best)

		list_drawer[i] = strategy_best.copy()
		if len(list_box_to_be_packed) == 0:
			break

	# 接口适配
	for i in range(len(list_drawer)):
		del list_drawer[i]['hive_id']
	num_drawer = len(list_hive[0]['drawers'])
	for i in range(len(list_hive)):
		list_hive[i]['drawers'] = list_drawer[i * num_drawer:(i + 1) * num_drawer]
	return list_hive

if __name__ == '__main__':
	list_box = [
		{"id": "1", "size": [10, 25, 20]},
		{"id": "2", "size": [30, 25, 20]},
		{"id": "3", "size": [20, 25, 20]},
		{"id": "4", "size": [20, 25, 30]},
		{"id": "5", "size": [30, 25, 20]},
		{"id": "6", "size": [25, 30, 20]},
		{"id": "7", "size": [30, 20, 25]},
		{"id": "8", "size": [25, 25, 20]},
		{"id": "9", "size": [20, 25, 30]},
		{"id": "10", "size": [30, 20, 25]},
		{"id": "11", "size": [30, 25, 20]},
		{"id": "12", "size": [15, 25, 20]}
	]

	list_hive = [
		{'id': '1',
		 'drawers': [
			 {'id': '1', 'dimensions': [60, 50, 40], 'boxes': []},
			 {'id': '2', 'dimensions': [60, 50, 40], 'boxes': []},
			 {'id': '3', 'dimensions': [60, 50, 40], 'boxes': []},
		 ]},
		{'id': '2',
		 'drawers': [
			 {'id': '1', 'dimensions': [60, 50, 40], 'boxes': []},
			 {'id': '2', 'dimensions': [60, 50, 40], 'boxes': []},
			 {'id': '3', 'dimensions': [60, 50, 40], 'boxes': []},
		 ]},
	]

	deadline = datetime.datetime.now() + datetime.timedelta(seconds=1)  # 截止时间

	ethnic_num = 20  # 族群规模
	pair_n = 10  # 配对次数
	variation_n = 1  # 变异次数

	selection_strategy(list_box, list_hive, deadline, ethnic_num=20, pair_n=10, variation_n=1)
	"""Runs a simple QProgEdit demonstration."""

	# validate.addPythonBuiltins(['builtin_var'])
	# app = QApplication(sys.argv)
	# 
	# treeWidgetItem1 = QTreeWidgetItem([u'Tab 1'])
	# treeWidgetItem3 = QTreeWidgetItem([u'Tab 3'])
	# symbolTree = QTreeWidget()
	# symbolTree.addTopLevelItem(treeWidgetItem1)
	# symbolTree.addTopLevelItem(treeWidgetItem3)
	# symbolTree.itemActivated.connect(activateSymbolTree)
	# 
	# tabManager = QTabManager(handlerButtonText=u'apply', runButton=True)
	# tabManager.setWindowIcon(QIcon.fromTheme(u'accessories-text-editor'))
	# tabManager.setWindowTitle(u'QProgEdit')
	# tabManager.resize(800, 600)
	# 
	# tabManager.cursorRowChanged.connect(cursorRowChanged)
	# tabManager.focusLost.connect(focusLost)
	# tabManager.focusReceived.connect(focusReceived)
	# tabManager.handlerButtonClicked.connect(handlerButtonClicked)
	# tabManager.execute.connect(runSelectedText)
	# 
	# tab = tabManager.addTab(u'Tab 1')
	# tab.setLang(u'Python')
	# tab.setSymbolTree(treeWidgetItem1)
	# tab.setText(open(__file__).read())
	# 
	# tab = tabManager.addTab(u'Tab 2')
	# tab.setText(u'Some plain text')
	# 
	# tab = tabManager.addTab(u'Tab 3')
	# tab.setLang(u'Python')
	# tab.setSymbolTree(treeWidgetItem3)
	# if os.path.exists(u'content.txt'):
	# 	tab.setText(open(u'content.txt').read())
	# 
	# layout = QHBoxLayout()
	# layout.addWidget(symbolTree)
	# layout.addWidget(tabManager)
	# container = QWidget()
	# container.setLayout(layout)
	# container.show()
	# 
	# res = app.exec_()
	# open(u'content.txt', u'w').write(tab.text())
	# sys.exit(res)
#
# from UtilSet import *
#
# if __name__ == '__main__':
# 	# main()
# 	# rm = [[ 9.999704e-01,  1.520084e-03, -7.537344e-03, -2.358491e+00],
# 	# 	 [-1.496757e-03 , 9.999941e-01 , 3.099550e-03 , 2.588960e-01],
# 	# 	 [ 7.542011e-03 ,-3.088177e-03 , 9.999668e-01,  2.091219e+00],
# 	# 	 [ 0.000000e+00 ,0.000000e+00,  0.000000e+00,  1.000000e+00]]
# 	# rm = [[0, 0, 0, 0.05],
#     #                        [0, -1, 0,0],
#     #                        [0, 0, -1, 0.05], [0, 0, 0, 1]]
# 	#
# 	# print(rm2rpy(rm))
# 	# rpy=[0.0362331333591835, 0.0927082892913209, 0.0231516312938686,0.0762413,0.0183472,6.27295757]
# 	# print(rpy2rm(rpy))
# 	rm = np.asarray([[9.99988512e-01, - 2.01915004e-03,  4.34717410e-03, - 1.66447867e-02],
# 	 [-2.05280673e-03, - 9.99967848e-01,  7.75170904e-03 , 9.82474297e-04],
# 	[4.33138246e-03, - 7.76054390e-03, - 9.99960506e-01 ,3.62461623e-02],
# 	[0.00000000e+00 , 0.00000000e+00,  0.00000000e+00 , 1.00000000e+00]])
# 	value = np.linalg.inv(rm)
# 	print(value)