let SessionLoad = 1
let s:so_save = &so | let s:siso_save = &siso | set so=0 siso=0
let v:this_session=expand("<sfile>:p")
silent only
cd ~/catkin_ws/src/open3d_workspace
if expand('%') == '' && !&modified && line('$') <= 1 && getline(1) == ''
  let s:wipebuf = bufnr('%')
endif
set shortmess=aoO
badd +1 term://.//3828:/bin/bash
badd +1 term://.//3806:/bin/bash
badd +1 term://.//3878:/bin/bash
badd +17 term://.//3932:/bin/bash
badd +1 README.md\[
badd +14 README.md
badd +1 term://.//4786:/bin/bash
badd +134 scripts/main.py
badd +11 term://.//23671:/bin/bash
badd +16 scripts/interactive_visualization.py
badd +4 term://.//5523:/bin/bash
badd +153 ~/catkin_ws/src/realsense2_camera/launch/rs_rgbd.launch
badd +35 ~/catkin_ws/src/realsense2_camera/cfg/origin_rs435_params.cfg
badd +1 scripts/reconfigure.sh
badd +436 reference.md
badd +1 hoge
badd +11 scripts/print_help.py
badd +31 term://.//13061:/bin/bash
badd +34 samples/scripts/removal.py
badd +77 term://.//15647:/bin/bash
badd +0 scripts/util.py
badd +0 samples/scripts/pointcloud.py
badd +13 samples/TestData/Crop/cropped.json
badd +0 samples/TestData/Crop/fragment.ply
badd +711 term://.//8013:/bin/bash
badd +50 samples/scripts/color_optimization.py
badd +0 term://.//22644:/bin/bash
badd +3 samples/scripts/trajectory_io.py
badd +1032 term://.//27832:/bin/bash
badd +28 term://.//3736:/bin/bash
badd +32 ~/numpy_train.py
badd +1 term://.//16700:/bin/bash
badd +514 term://.//21051:/bin/bash
badd +0 ~/catkin_ws/src/o_gpsr_2018/scripts/google_tts.py
badd +0 ~/catkin_ws/src/o_display_disporsal_2018/scripts/main.py
badd +0 term://.//4189:/bin/bash
argglobal
silent! argdel *
edit scripts/util.py
set splitbelow splitright
wincmd _ | wincmd |
split
1wincmd k
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
wincmd w
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 106 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
exe '3resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 106 + 106) / 212)
exe '4resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 105 + 106) / 212)
argglobal
if bufexists('term://.//3806:/bin/bash') | buffer term://.//3806:/bin/bash | else | edit term://.//3806:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1328 - ((28 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1328
normal! 09|
wincmd w
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 61 - ((19 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
61
normal! 018|
wincmd w
argglobal
if bufexists('term://.//3878:/bin/bash') | buffer term://.//3878:/bin/bash | else | edit term://.//3878:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1 - ((0 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1
normal! 0
wincmd w
argglobal
if bufexists('~/catkin_ws/src/o_gpsr_2018/scripts/google_tts.py') | buffer ~/catkin_ws/src/o_gpsr_2018/scripts/google_tts.py | else | edit ~/catkin_ws/src/o_gpsr_2018/scripts/google_tts.py | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 91 - ((15 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
91
normal! 05|
wincmd w
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 106 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
exe '3resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 106 + 106) / 212)
exe '4resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 105 + 106) / 212)
tabedit scripts/main.py
set splitbelow splitright
wincmd _ | wincmd |
split
1wincmd k
wincmd _ | wincmd |
vsplit
wincmd _ | wincmd |
vsplit
2wincmd h
wincmd w
wincmd w
wincmd w
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe '1resize ' . ((&lines * 11 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 70 + 106) / 212)
exe '2resize ' . ((&lines * 11 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 70 + 106) / 212)
exe '3resize ' . ((&lines * 11 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 70 + 106) / 212)
exe '4resize ' . ((&lines * 47 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 106 + 106) / 212)
exe '5resize ' . ((&lines * 47 + 31) / 62)
exe 'vert 5resize ' . ((&columns * 105 + 106) / 212)
argglobal
if bufexists('term://.//21051:/bin/bash') | buffer term://.//21051:/bin/bash | else | edit term://.//21051:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 546 - ((10 * winheight(0) + 5) / 11)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
546
normal! 040|
wincmd w
argglobal
if bufexists('term://.//27832:/bin/bash') | buffer term://.//27832:/bin/bash | else | edit term://.//27832:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1036 - ((10 * winheight(0) + 5) / 11)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1036
normal! 040|
wincmd w
argglobal
if bufexists('term://.//8013:/bin/bash') | buffer term://.//8013:/bin/bash | else | edit term://.//8013:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 2035 - ((10 * winheight(0) + 5) / 11)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
2035
normal! 0
wincmd w
argglobal
if bufexists('term://.//4189:/bin/bash') | buffer term://.//4189:/bin/bash | else | edit term://.//4189:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 70 - ((13 * winheight(0) + 23) / 47)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
70
normal! 0
wincmd w
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 145 - ((29 * winheight(0) + 23) / 47)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
145
normal! 016|
wincmd w
5wincmd w
exe '1resize ' . ((&lines * 11 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 70 + 106) / 212)
exe '2resize ' . ((&lines * 11 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 70 + 106) / 212)
exe '3resize ' . ((&lines * 11 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 70 + 106) / 212)
exe '4resize ' . ((&lines * 47 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 106 + 106) / 212)
exe '5resize ' . ((&lines * 47 + 31) / 62)
exe 'vert 5resize ' . ((&columns * 105 + 106) / 212)
tabedit reference.md
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 125 + 106) / 212)
exe 'vert 2resize ' . ((&columns * 86 + 106) / 212)
argglobal
if bufexists('term://.//13061:/bin/bash') | buffer term://.//13061:/bin/bash | else | edit term://.//13061:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 49 - ((3 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
49
normal! 07|
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 190 - ((20 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
190
normal! 063|
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
exe 'vert 1resize ' . ((&columns * 125 + 106) / 212)
exe 'vert 2resize ' . ((&columns * 86 + 106) / 212)
tabedit ~/catkin_ws/src/o_display_disporsal_2018/scripts/main.py
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 105 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
exe 'vert 3resize ' . ((&columns * 106 + 106) / 212)
argglobal
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 6 - ((5 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 0
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
argglobal
if bufexists('term://.//22644:/bin/bash') | buffer term://.//22644:/bin/bash | else | edit term://.//22644:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 91 - ((27 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
91
normal! 0
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
argglobal
if bufexists('~/catkin_ws/src/open3d_workspace/samples/scripts/pointcloud.py') | buffer ~/catkin_ws/src/open3d_workspace/samples/scripts/pointcloud.py | else | edit ~/catkin_ws/src/open3d_workspace/samples/scripts/pointcloud.py | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 2 - ((1 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
2
normal! 0
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 105 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
exe 'vert 3resize ' . ((&columns * 106 + 106) / 212)
tabnext 2
if exists('s:wipebuf') && getbufvar(s:wipebuf, '&buftype') isnot# 'terminal'
  silent exe 'bwipe ' . s:wipebuf
endif
unlet! s:wipebuf
set winheight=1 winwidth=20 winminheight=1 winminwidth=1 shortmess=filnxtToO
let s:sx = expand("<sfile>:p:r")."x.vim"
if file_readable(s:sx)
  exe "source " . fnameescape(s:sx)
endif
let &so = s:so_save | let &siso = s:siso_save
doautoall SessionLoadPost
unlet SessionLoad
" vim: set ft=vim :
