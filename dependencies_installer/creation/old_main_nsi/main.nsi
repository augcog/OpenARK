; Start
  !include LogicLib.nsh
  !include "winmessages.nsh"
  !include "MUI.nsh"
  !define MUI_NAME "OpenARK Dependency Pack"
  !define MUI_FILE "savefile"
  !define MUI_BRANDINGTEXT "OpenARK"
  !define MUI_WELCOMEPAGE_TITLE "Welcome to the OpenARK Dependency Pack Installer"
  !define MUI_LICENSEPAGE_TEXT_TOP "Please review the note and license terms before installing"
  !define MUI_WELCOMEFINISHPAGE_BITMAP "welcome.bmp"
  Name "OpenARK Dependency Pack"
  CRCCheck On

  ;!include "${NSISDIR}\Contrib\Modern UI\System.nsh"

;---------------------------------
;General

  OutFile "openark-deps-vc14-win64.exe"
  ShowInstDetails "nevershow"
  ShowUninstDetails "nevershow"

  SetCompressor /SOLID lzma
  SetCompressorDictSize 64
  SetDatablockOptimize ON

  !define MUI_ICON "icon.ico"
  !define MUI_UNICON "icon.ico"
  ;!define MUI_SPECIALBITMAP "Bitmap.bmp"

;--------------------------------
;Folder selection page

  InstallDir "$PROGRAMFILES64\OpenARK-Deps"


;--------------------------------
;Modern UI Configuration

  !insertmacro MUI_PAGE_WELCOME
  !insertmacro MUI_PAGE_LICENSE "license.txt"
  !insertmacro MUI_PAGE_DIRECTORY
  !insertmacro MUI_PAGE_INSTFILES
  !define MUI_ABORTWARNING
  !insertmacro MUI_PAGE_FINISH

  ;!insertmacro MUI_UNPAGE_WELCOME
  !insertmacro MUI_UNPAGE_CONFIRM
  !insertmacro MUI_UNPAGE_INSTFILES
  !insertmacro MUI_UNPAGE_FINISH


;--------------------------------
;Language

  !insertmacro MUI_LANGUAGE "English"

;--------------------------------
;Data

  LicenseData "license.txt"

;--------------------------------
;Installer Sections
Section "install" ;Installation info

;Add files
  !include()
  SetOutPath "$INSTDIR"
  File /R "..\arkdeps\*"

;write uninstall information to the registry
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\openark-deps" "DisplayName" "${MUI_NAME} (remove only)"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\openark-deps" "UninstallString" "$INSTDIR\Uninstall.exe"
  WriteRegStr HKLM "SYSTEM\CurrentControlSet\Control\Session Manager\Environment" "ARK_DEPS_DIR" "$INSTDIR"
  ;SendMessage ${HWND_BROADCAST} ${WM_WININICHANGE} 0 "STR:Environment" /TIMEOUT=5000

  EnVar::SetHKCU
  EnVar::AddValue "PATH" "$INSTDIR\extra\bin"

  WriteUninstaller "$INSTDIR\extra\Uninstall.exe"

;Rearrange Files
  Rename "$INSTDIR\CMakeLists.txt" "$INSTDIR\extra\CMakeLists.txt"

SectionEnd

;--------------------------------
;Uninstaller user check Function
Function checkUninstallFilesWithUser
  MessageBox MB_OK "OpenARK installed files that will be uninstalled: "
  MessageBox MB_YESNO "Are you sure you want to uninstall these files?" IDYES NoAbort
    DetailPrint "Cancelling..."
    Abort
  NoAbort:

  ;Delete Files
  ;Replacing: RMDir /r "$INSTDIR\*.*"
  ;if check to see if all of these folders exist, only then should I remove these. if not, then return error message saying uinstall should stop
  ;bin, cmakelist, and uninstall - we should create a separate directory/folder to contain these three. Then, we avoid the risk of deleting the wrong bin
  ;maybe just adding a method for checking if the user wants to delete these yes/no. list all the files/locations that would be uninstalled, similar to apt-install log

  ; simply listing file names for deleting and yes/no choices is all I need

  RMDir /r "$INSTDIR\bin"
  RMDir /r "$INSTDIR\brisk"
  RMDir /r "$INSTDIR\Ceres"
  RMDir /r "$INSTDIR\DBoW2"
  RMDir /r "$INSTDIR\DLib"
  RMDir /r "$INSTDIR\DLoopDetector"
  RMDir /r "$INSTDIR\eigen3"
  RMDir /r "$INSTDIR\glog"
  RMDir /r "$INSTDIR\librealsense2"
  RMDir /r "$INSTDIR\okvis"
  RMDir /r "$INSTDIR\Open3D"
  RMDir /r "$INSTDIR\opencv343-contrib"
  RMDir /r "$INSTDIR\opengv"
  RMDir /r "$INSTDIR\suitesparse"
  RMDir /r "$INSTDIR\CMakeLists.txt"
  RMDir /r "$INSTDIR\Uninstall.exe"

  ;Remove the installation directory
  RMDir "$INSTDIR"

  ;Delete Start Menu Shortcuts
  ;Delete "$DESKTOP\${MUI_NAME}.lnk"
  ;Delete "$SMPROGRAMS\${MUI_NAME}\*.*"
  ;RmDir  "$SMPROGRAMS\${MUI_NAME}"

  ;Delete Uninstaller And Unistall Registry Entries
  DeleteRegKey HKLM "SOFTWARE\openark-deps"
  DeleteRegKey HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\openark-deps"
  DeleteRegValue HKLM "SYSTEM\CurrentControlSet\Control\Session Manager\Environment" "ARK_DEPS_DIR"
  ;SendMessage ${HWND_BROADCAST} ${WM_WININICHANGE} 0 "STR:Environment" /TIMEOUT=5000

  EnVar::SetHKCU
  EnVar::DeleteValue "PATH" "$INSTDIR\bin"
FunctionEnd

;Uninstaller Section

Section "Uninstall"
  Call checkUninstallFilesWithUser
SectionEnd


;--------------------------------
;MessageBox Section


;Function that calls a messagebox when installation finished correctly
Function .onInstSuccess
  MessageBox MB_OK "You have successfully installed the ${MUI_NAME}. Please proceed to configure OpenARK using cmake .. -G'Visual Studio 14 2015 Win64'"
FunctionEnd


Function un.onUninstSuccess
  MessageBox MB_OK "You have successfully uninstalled the ${MUI_NAME}."
FunctionEnd

;eof
