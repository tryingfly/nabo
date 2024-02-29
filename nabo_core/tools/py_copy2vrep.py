'''=========== ***doc description @ yyp*** ===========
This is part of Nabo (Naughty Boy, 小炮), an open project for the control of biped robot，
Copyright (C) 2024 YYP with MIT License.
Feel free to use in any purpose, and cite Nabo or 小炮 in any style, to contribut to the advancement of the community.
<https://github.com/tryingfly/nabo>
<https://www.zhihu.com/column/c_1746480685987373057>

======================================================'''
import sys
import shutil

if sys.platform.startswith('win'):
	shutil.copy('build/Release/nabo.dll', '../nabo_coppelia')
else:
	shutil.copy('build/libnabo.so', '../nabo_coppelia')

