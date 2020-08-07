"""
This script generates 2 lists of NSIS commands (install&uninstall)
for all files in a given directory

Usage:
    gen_list_files_for_nsis.py  <dir src> <inst list> <uninst list>
Where
    <dir src>       :   dir with sources; must exist
    <inst list>     :   list of files to install (NSIS syntax)
    <uninst list>   :   list of files to uninstall (NSIS syntax)
    <file list>     :   prints list of files (NSIS syntax)
                        (all of these will be overwriten each time)
"""
import sys, os, glob

# templates for the output
inst_dir_tpl  = '  SetOutPath "$INSTDIR%s"'
inst_file_tpl = '  File "%s"'
uninst_file_tpl = '  Delete "$INSTDIR%s"'
uninst_dir_tpl  = '  RMDir "$INSTDIR%s"'
uninst_dir_rec_tpl = '  RMDir /r "$INSTDIR\%s"'
file_list_tpl = '  MessageBox MB_OK "$INSTDIR\%s"'

# check args
if len(sys.argv) != 5:
    print(__doc__)
    sys.exit(1)
source_dir = sys.argv[1]
if not os.path.isdir(source_dir):
    print(__doc__)
    sys.exit(1)

def open_file_for_writing(filename):
    "return a handle to the file to write to"
    try:
        h = open(filename, "w")
    except:
        print("Problem opening file %s for writing"%filename)
        print(__doc__)
        sys.exit(1)
    return h

inst_list = sys.argv[2]
uninst_list = sys.argv[3]
file_list = sys.argv[4]

ih= open_file_for_writing(inst_list)
uh= open_file_for_writing(uninst_list)
fl = open_file_for_writing(file_list)

stack_of_visited = []
counter_files = 0
counter_dirs = 0
print("Generating the install & uninstall list of files")
print("  for directory", source_dir)
print("  ; Files to install\n", file=ih)
print("  ; Files and folders to remove\n", file=uh)

for path, dirnames, filenames in os.walk(source_dir):
    if counter_dirs == 0:
        for d in dirnames:
            print(uninst_dir_rec_tpl % d, file=uh)
            print("  ", file=uh)
            print(file_list_tpl % d, file=fl)
            print("  ", file=fl)


    counter_dirs += 1
    my_dir = path
    if len(dirnames) == 0:
        relative_path = path[len("../arkdeps"):]
        print(inst_dir_tpl % relative_path, file=ih)
        print("  ", file=ih)
        stack_of_visited.append( ('folder', path, my_dir) )
        for f in filenames:
            my_file = os.path.join(my_dir, f)
            print(inst_file_tpl % my_file, file=ih)
            print("  ", file=ih)
            counter_files += 1
            stack_of_visited.append( ('file', my_file, my_dir) )

ih.close()
print("Install list done")
print("  ", counter_files, "files in", counter_dirs, "dirs")

# stack_of_visited.reverse()
# # Now build the uninstall list
# for (type, my_file_or_dir, my_dir) in stack_of_visited:
#     if type == 'file':
#         file_name = my_file_or_dir[len('../arkdeps'):]
#         print(uninst_file_tpl % file_name, file=uh)
#         print(file_list_tpl % file_name, file=fl)
#     else:
#         dir_name = my_file_or_dir[len('../arkdeps'):]
#         print(uninst_dir_tpl % dir_name, file=uh)
#         print(file_list_tpl % dir_name, file=fl)
#     print("  ", file=uh)

# now close everything
uh.close()
fl.close()
print("Uninstall list done. Got to end.\n")
