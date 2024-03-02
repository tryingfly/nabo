'''=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP, Shanghai Jiao Tong University, under MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

======================================================'''
import sys
import shutil

if sys.platform.startswith('win'):
	shutil.copy('build/Release/nabo.lib', '../nabo_mujoco/nabo')
	shutil.copy('build/Release/nabo.dll', '../nabo_mujoco/nabo')
	shutil.copy('build/Release/nabo.dll', '../nabo_mujoco/build/Release')
else:
	shutil.copy('build/nabo.so', '../nabo_mujoco/nabo')
	shutil.copy('build/nabo.so', '../nabo_mujoco/build')

