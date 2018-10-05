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
badd +1 term://.//3932:/bin/bash
badd +1 term://.//4809:/bin/bash
badd +1 README.md\[
badd +7 README.md
badd +1 term://.//4786:/bin/bash
badd +1 scripts/main.py
badd +11 term://.//23671:/bin/bash
badd +16 scripts/interactive_visualization.py
badd +4 term://.//5523:/bin/bash
badd +153 ~/catkin_ws/src/realsense2_camera/launch/rs_rgbd.launch
badd +35 ~/catkin_ws/src/realsense2_camera/cfg/origin_rs435_params.cfg
badd +1 scripts/reconfigure.sh
badd +1 term://.//4883:/bin/bash
badd +436 reference.md
badd +1 hoge
badd +0 term://.//9000:/bin/bash
badd +11 scripts/print_help.py
badd +31 term://.//13061:/bin/bash
badd +1 samples/scripts/removal.py
badd +0 term://.//15647:/bin/bash
badd +0 scripts/util.py
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
let s:l = 4 - ((3 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
4
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
let s:l = 61 - ((26 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
61
normal! 05|
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
if bufexists('term://.//3932:/bin/bash') | buffer term://.//3932:/bin/bash | else | edit term://.//3932:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 6 - ((3 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
6
normal! 0
wincmd w
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 106 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
exe '3resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 106 + 106) / 212)
exe '4resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 105 + 106) / 212)
tabedit README.md
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd _ | wincmd |
split
1wincmd k
wincmd w
wincmd w
wincmd _ | wincmd |
split
1wincmd k
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 55 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 55 + 106) / 212)
exe '3resize ' . ((&lines * 14 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 70 + 106) / 212)
exe '4resize ' . ((&lines * 14 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 85 + 106) / 212)
exe '5resize ' . ((&lines * 44 + 31) / 62)
exe 'vert 5resize ' . ((&columns * 156 + 106) / 212)
argglobal
if bufexists('term://.//9000:/bin/bash') | buffer term://.//9000:/bin/bash | else | edit term://.//9000:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 182 - ((28 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
182
normal! 0
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
argglobal
if bufexists('term://.//4809:/bin/bash') | buffer term://.//4809:/bin/bash | else | edit term://.//4809:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 1758 - ((28 * winheight(0) + 14) / 29)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
1758
normal! 0
wincmd w
argglobal
if bufexists('term://.//4883:/bin/bash') | buffer term://.//4883:/bin/bash | else | edit term://.//4883:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 65 - ((13 * winheight(0) + 7) / 14)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
65
normal! 08|
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
let s:l = 9 - ((8 * winheight(0) + 7) / 14)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
9
normal! 0
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
argglobal
if bufexists('~/catkin_ws/src/open3d_workspace/scripts/main.py') | buffer ~/catkin_ws/src/open3d_workspace/scripts/main.py | else | edit ~/catkin_ws/src/open3d_workspace/scripts/main.py | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
silent! normal! zE
let s:l = 243 - ((28 * winheight(0) + 22) / 44)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
243
normal! 038|
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
2wincmd w
exe '1resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 1resize ' . ((&columns * 55 + 106) / 212)
exe '2resize ' . ((&lines * 29 + 31) / 62)
exe 'vert 2resize ' . ((&columns * 55 + 106) / 212)
exe '3resize ' . ((&lines * 14 + 31) / 62)
exe 'vert 3resize ' . ((&columns * 70 + 106) / 212)
exe '4resize ' . ((&lines * 14 + 31) / 62)
exe 'vert 4resize ' . ((&columns * 85 + 106) / 212)
exe '5resize ' . ((&lines * 44 + 31) / 62)
exe 'vert 5resize ' . ((&columns * 156 + 106) / 212)
tabedit ~/catkin_ws/src/open3d_workspace/reference.md
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
let s:l = 80 - ((58 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
80
normal! 0
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
let s:l = 182 - ((18 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
182
normal! 09|
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
exe 'vert 1resize ' . ((&columns * 125 + 106) / 212)
exe 'vert 2resize ' . ((&columns * 86 + 106) / 212)
tabedit ~/catkin_ws/src/open3d_workspace/samples/scripts/removal.py
set splitbelow splitright
wincmd _ | wincmd |
vsplit
1wincmd h
wincmd w
set nosplitbelow
set nosplitright
wincmd t
set winminheight=1 winminwidth=1 winheight=1 winwidth=1
exe 'vert 1resize ' . ((&columns * 106 + 106) / 212)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
argglobal
if bufexists('term://.//15647:/bin/bash') | buffer term://.//15647:/bin/bash | else | edit term://.//15647:/bin/bash | endif
setlocal fdm=manual
setlocal fde=0
setlocal fmr={{{,}}}
setlocal fdi=#
setlocal fdl=0
setlocal fml=1
setlocal fdn=20
setlocal fen
let s:l = 45 - ((44 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
45
normal! 010|
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
let s:l = 33 - ((32 * winheight(0) + 29) / 59)
if s:l < 1 | let s:l = 1 | endif
exe s:l
normal! zt
33
normal! 05|
lcd ~/catkin_ws/src/open3d_workspace
wincmd w
exe 'vert 1resize ' . ((&columns * 106 + 106) / 212)
exe 'vert 2resize ' . ((&columns * 105 + 106) / 212)
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
