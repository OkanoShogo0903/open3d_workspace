#!/usr/bin/python
# -*- coding: utf-8 -*-

import open3d
#import pathlib
#p_new = pathlib.Path('open3d_reference.txt')

file_ = open('open3d_reference.txt', 'w')  #書き込みモードでオープン
string = help(open3d)
 
#file_.write(string)

# if p_new.exists() == True:
#     with p_new.open(mode='w') as f:
#         f.write( help(open3d) )
# else:
#     pass
