; installer.iss — "Airlive for OBS" for Windows (Inno Setup 6).
;
; Installs into OBS's machine-wide plugin folder, %ProgramData%\obs-studio\plugins\airlive-obs,
; which every OBS 30+ loads (bin\64bit\ for the module, data\ for its resources). Needs admin:
; ProgramData is shared by every user of the PC. Everything shown to the user is English.
;
; Build: iscc /DAppVersion=1.4.0 /DBuildDir=<folder with airlive-obs.dll> /DRepoDir=<repo root> installer.iss

#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif

[Setup]
AppId={{43D263FE-1806-48F3-B595-800D5F9AAF46}
AppName=Airlive for OBS
AppVersion={#AppVersion}
AppPublisher=Airlive
AppPublisherURL=https://airliveproject.com
DefaultDirName={commonappdata}\obs-studio\plugins\airlive-obs
DisableDirPage=yes
DisableProgramGroupPage=yes
PrivilegesRequired=admin
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
; Windows 10 1809: the first with the built-in mDNS advertising the plugin uses.
MinVersion=10.0.17763
OutputBaseFilename=Airlive-for-OBS-{#AppVersion}-Windows
Compression=lzma2
SolidCompression=yes
UninstallDisplayName=Airlive for OBS
CloseApplications=no
WizardStyle=modern

[Files]
Source: "{#BuildDir}\airlive-obs.dll"; DestDir: "{app}\bin\64bit"; Flags: ignoreversion
Source: "{#RepoDir}\data\*"; DestDir: "{app}\data"; Flags: ignoreversion recursesubdirs

[Messages]
WelcomeLabel2=This installs the Airlive plugin for OBS Studio.%n%nClose OBS before you continue. When it finishes, start OBS and add a source with + : Airlive Camera, an iPhone running the Airlive Camera app.
FinishedLabel=Installation complete.%n%nStart OBS Studio - Airlive Camera is in the + (Add Source) menu.%n%nThe first time a camera connects, Windows may ask whether OBS may accept connections on your network. Allow it on private networks, or the iPhone cannot reach OBS.
