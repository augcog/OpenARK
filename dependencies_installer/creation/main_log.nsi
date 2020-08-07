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

  !include  ${INST_LIST} ; the payload of this installer is described in an externally generated list of files

;File /r "${FILES_SOURCE_PATH}\*.*" ; not OK because with /r we can not log what was installed
                                      ; and without logging we cannot uninstall only the files installed by us

;write uninstall information to the registry
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\openark-deps" "DisplayName" "${MUI_NAME} (remove only)"
  WriteRegStr HKLM "Software\Microsoft\Windows\CurrentVersion\Uninstall\openark-deps" "UninstallString" "$INSTDIR\Uninstall.exe"
  WriteRegStr HKLM "SYSTEM\CurrentControlSet\Control\Session Manager\Environment" "ARK_DEPS_DIR" "$INSTDIR"
  ;SendMessage ${HWND_BROADCAST} ${WM_WININICHANGE} 0 "STR:Environment" /TIMEOUT=5000

  EnVar::SetHKCU
  EnVar::AddValue "PATH" "$INSTDIR\extra\bin"

  WriteUninstaller "$INSTDIR\Uninstall.exe"

SectionEnd

;--------------------------------
;
;  uninstaller
;
;--------------------------------

Section "Uninstall"
  MessageBox MB_OK "OpenARK installed files that will be uninstalled: "
  !include ${FILE_LIST}

  MessageBox MB_YESNO "Are you sure you want to uninstall these files?" IDYES NoAbort
    DetailPrint "Cancelling..."
    Abort
  NoAbort:

  ; Remove the files (using externally generated file list)
  !include ${UNINST_LIST}

  ; Remove uninstaller
  Delete $INSTDIR\uninst*.exe

  RMDir $INSTDIR ; this is safe; it's not forced if you still have private files there.

  ; Important note: RMDir /r "$INSTDIR"  ; is NOT OK!
  ; if the user installed in "C:\Program Files" by mistake, then we totally screw up his machine!

  ;Delete Uninstaller And Unistall Registry Entries
  DeleteRegKey HKLM "SOFTWARE\openark-deps"
  DeleteRegKey HKLM "SOFTWARE\Microsoft\Windows\CurrentVersion\Uninstall\openark-deps"
  DeleteRegValue HKLM "SYSTEM\CurrentControlSet\Control\Session Manager\Environment" "ARK_DEPS_DIR"
  ;SendMessage ${HWND_BROADCAST} ${WM_WININICHANGE} 0 "STR:Environment" /TIMEOUT=5000

  EnVar::SetHKCU
  EnVar::DeleteValue "PATH" "$INSTDIR\extra\bin"

SectionEnd ; Uninstall

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
