// 
// Updates an ApplicationHost.config file in the current user's
// profile directory from IIS 7.5 Express to IIS 10.0 Express.
// 

var strCommitPath = "MACHINE/WEBROOT/APPHOST";
var strFirstItem  = "##FIRST#ITEM##";
var strLastItem   = "##LAST#ITEM##";

// ------------------------------------------------------------
// Check for an existing ApplicationHost.config file.
// ------------------------------------------------------------

var objFSO = new ActiveXObject("scripting.filesystemobject");
var strUserConfig = GetUserDirectory() + "\\config\\ApplicationHost.config";
if (objFSO.FileExists(strUserConfig))
{
	// Backup existing an existing ApplicationHost.config file.
	try
	{
		var dtmDate = new Date();
		var strDate = dtmDate.getYear().toString() + 
			PadNumber(dtmDate.getMonth()+1) + 
			PadNumber(dtmDate.getDate()) + 
			PadNumber(dtmDate.getHours()) + 
			PadNumber(dtmDate.getMinutes()) + 
			PadNumber(dtmDate.getSeconds());
	 	objFSO.CopyFile(strUserConfig,strUserConfig + "." + strDate + ".bak",true);
	}
	catch(e)
	{
		ErrorMessage(e,"An error occurred trying to back up your ApplicationHost.config file");
	}
}
else
{
	// Exit if no applicationhost.config file exists. (This is not an error condition.)
	WScript.Echo("No ApplicationHost.config file exists in the current user's profile directory - exiting.");
	WScript.Quit(0);
}

// ------------------------------------------------------------
// Retrieve the necessary objects for the rest of the script.
// ------------------------------------------------------------

WScript.Echo("Migrating your ApplicationHost.config file...\n");
var objAdminManager      = GetAdminManager();
var objConfigManager     = objAdminManager.ConfigManager;
var objAppHostConfig     = objConfigManager.GetConfigFile(strCommitPath);
var objRootSectionGroup  = objAppHostConfig.RootSectionGroup;

// ------------------------------------------------------------
WScript.Echo("...adding new section groups...");
// ------------------------------------------------------------

var objSystemWebServer = FindSectionGroup(objRootSectionGroup,"system.webServer");
AddSection(objSystemWebServer,"applicationInitialization","Allow","MachineToApplication","");
AddSection(objSystemWebServer,"webSocket","Deny","","");
var objSecurity = FindSectionGroup(objSystemWebServer,"security");
AddSection(objSecurity,"dynamicIpSecurity","Deny","","");

// ------------------------------------------------------------
WScript.Echo("...adding new global modules...");
// ------------------------------------------------------------

var objGlobalModules = objAdminManager.GetAdminSection("system.webServer/globalModules", strCommitPath);
AddGlobalModule(objGlobalModules.Collection,"DynamicIpRestrictionModule","%IIS_BIN%\\diprestr.dll","","IpRestrictionModule");
AddGlobalModule(objGlobalModules.Collection,"ApplicationInitializationModule","%IIS_BIN%\\warmup.dll","","ConfigurationValidationModule");
AddGlobalModule(objGlobalModules.Collection,"WebSocketModule","%IIS_BIN%\\iiswsock.dll","","ApplicationInitializationModule");
AddGlobalModule(objGlobalModules.Collection,"ManagedEngine64","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\webengine.dll","integratedMode,runtimeVersionv2.0,bitness64","ManagedEngine");
AddGlobalModule(objGlobalModules.Collection,"ManagedEngineV4.0_64bit","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\webengine4.dll","integratedMode,runtimeVersionv4.0,bitness64","ManagedEngineV4.0_32bit");

// ------------------------------------------------------------
WScript.Echo("...adding new ISAPI filters...");
// ------------------------------------------------------------

var objIsapiFilters = objAdminManager.GetAdminSection("system.webServer/isapiFilters", strCommitPath);
AddIsapiFilter(objIsapiFilters.Collection,"ASP.Net_2.0.50727-64","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_filter.dll","","true","bitness64,runtimeVersionv2.0",strFirstItem);
AddIsapiFilter(objIsapiFilters.Collection,"ASP.Net_4.0_64bit","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_filter.dll","","true","bitness64,runtimeVersionv4.0","ASP.Net_4.0_32bit");

// ------------------------------------------------------------
WScript.Echo("...adding new ISAPI/CGI restrictions...");
// ------------------------------------------------------------

var objIsapiCgiRestrictions = objAdminManager.GetAdminSection("system.webServer/security/isapiCgiRestriction", strCommitPath);
AddIsapiCgiRestriction(objIsapiCgiRestrictions.Collection,"%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\webengine4.dll","true","ASP.NET_v4.0","ASP.NET_v4.0",strFirstItem);
AddIsapiCgiRestriction(objIsapiCgiRestrictions.Collection,"%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","true","ASP.NET v2.0.50727","ASP.NET v2.0.50727",strLastItem);
AddIsapiCgiRestriction(objIsapiCgiRestrictions.Collection,"%windir%\\Microsoft.NET\\Framework\\v2.0.50727\\aspnet_isapi.dll","true","ASP.NET v2.0.50727","ASP.NET v2.0.50727",strLastItem);

// ------------------------------------------------------------
WScript.Echo("...adding new MIME maps...");
// ------------------------------------------------------------

var objStaticContent = objAdminManager.GetAdminSection("system.webServer/staticContent", strCommitPath);
AddMimeMap(objStaticContent.Collection,".3g2","video/3gpp2");
AddMimeMap(objStaticContent.Collection,".3gp2","video/3gpp2");
AddMimeMap(objStaticContent.Collection,".3gp","video/3gpp");
AddMimeMap(objStaticContent.Collection,".3gpp","video/3gpp");
AddMimeMap(objStaticContent.Collection,".aac","audio/aac");
AddMimeMap(objStaticContent.Collection,".adt","audio/vnd.dlna.adts");
AddMimeMap(objStaticContent.Collection,".adts","audio/vnd.dlna.adts");
AddMimeMap(objStaticContent.Collection,".cab","application/vnd.ms-cab-compressed");
AddMimeMap(objStaticContent.Collection,".dvr-ms","video/x-ms-dvr");
AddMimeMap(objStaticContent.Collection,".eot","application/vnd.ms-fontobject");
AddMimeMap(objStaticContent.Collection,".js","application/javascript");
AddMimeMap(objStaticContent.Collection,".m2ts","video/vnd.dlna.mpeg-tts");
AddMimeMap(objStaticContent.Collection,".m4a","audio/mp4");
AddMimeMap(objStaticContent.Collection,".m4v","video/mp4");
AddMimeMap(objStaticContent.Collection,".mp4","video/mp4");
AddMimeMap(objStaticContent.Collection,".mp4v","video/mp4");
AddMimeMap(objStaticContent.Collection,".oga","audio/ogg");
AddMimeMap(objStaticContent.Collection,".ogg","video/ogg");
AddMimeMap(objStaticContent.Collection,".ogv","video/ogg");
AddMimeMap(objStaticContent.Collection,".ogx","application/ogg");
AddMimeMap(objStaticContent.Collection,".otf","font/otf");
AddMimeMap(objStaticContent.Collection,".spx","audio/ogg");
AddMimeMap(objStaticContent.Collection,".svg","image/svg+xml");
AddMimeMap(objStaticContent.Collection,".svgz","image/svg+xml");
AddMimeMap(objStaticContent.Collection,".ts","video/vnd.dlna.mpeg-tts");
AddMimeMap(objStaticContent.Collection,".tts","video/vnd.dlna.mpeg-tts");
AddMimeMap(objStaticContent.Collection,".webm","video/webm");
AddMimeMap(objStaticContent.Collection,".woff","font/x-woff");
AddMimeMap(objStaticContent.Collection,".wtv","video/x-ms-wtv");
AddMimeMap(objStaticContent.Collection,".xht","application/xhtml+xml");
AddMimeMap(objStaticContent.Collection,".xhtml","application/xhtml+xml");

// ------------------------------------------------------------
WScript.Echo("...adding new trace provider definitions...");
// ------------------------------------------------------------

var objTraceProviderDefinitions = objAdminManager.GetAdminSection("system.webServer/tracing/traceProviderDefinitions", strCommitPath);
AddTraceProviderDefinitions(objTraceProviderDefinitions.Collection,"WWW Server","WebSocket","16384");

// ------------------------------------------------------------
WScript.Echo("...updating trace areas...");
// ------------------------------------------------------------

var objTraceAreas = objAdminManager.GetAdminSection("system.webServer/tracing/traceFailedRequests", strCommitPath);
UpdateTraceAreas(objTraceAreas.Collection,"WWW Server","Authentication,Security,Filter,StaticFile,CGI,Compression,Cache,RequestNotifications,Module,Rewrite,WebSocket","Verbose");

// ------------------------------------------------------------
WScript.Echo("...updating WebDAV global settings...");
// ------------------------------------------------------------

var objWebDavGlobalSettings = objAdminManager.GetAdminSection("system.webServer/webdav/globalSettings", strCommitPath);
UpdateWebDavGlobalSettings(objWebDavGlobalSettings.ChildElements.Item("propertyStores").Collection,"webdav_simple_prop","%IIS_BIN%\\webdav_simple_prop.dll","%IIS_BIN%\\webdav_simple_prop.dll");
UpdateWebDavGlobalSettings(objWebDavGlobalSettings.ChildElements.Item("lockStores").Collection,"webdav_simple_lock","%IIS_BIN%\\webdav_simple_lock.dll","%IIS_BIN%\\webdav_simple_lock.dll");

// ------------------------------------------------------------
WScript.Echo("...adding new modules...");
// ------------------------------------------------------------

var objModules = objAdminManager.GetAdminSection("system.webServer/modules", strCommitPath);
AddModule(objModules.Collection,"WebMatrixSupportModule","true","","","IISCertificateMappingAuthenticationModule");
AddModule(objModules.Collection,"DynamicIpRestrictionModule","true","","","IpRestrictionModule");
AddModule(objModules.Collection,"ApplicationInitializationModule","true","","","UrlMappingsModule");
AddModule(objModules.Collection,"WebSocketModule","true","","","ApplicationInitializationModule");
AddModule(objModules.Collection,"ConfigurationValidationModule","true","","","ServiceModel-4.0");

// ------------------------------------------------------------
WScript.Echo("...adding new handlers...");
// ------------------------------------------------------------

var objHandlers = objAdminManager.GetAdminSection("system.webServer/handlers", strCommitPath);
AddHandler(objHandlers.Collection,"vbhtml-ISAPI-4.0_64bit","*.vbhtml","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"vbhtm-ISAPI-4.0_64bit","*.vbhtm","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"cshtml-ISAPI-4.0_64bit","*.cshtml","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"cshtm-ISAPI-4.0_64bit","*.cshtm","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"aspq-ISAPI-4.0_64bit","*.aspq","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"xamlx-ISAPI-4.0_64bit","*.xamlx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","","",strFirstItem);
AddHandler(objHandlers.Collection,"xoml-ISAPI-4.0_64bit","*.xoml","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","","",strFirstItem);
AddHandler(objHandlers.Collection,"rules-ISAPI-4.0_64bit","*.rules","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","","",strFirstItem);
AddHandler(objHandlers.Collection,"svc-ISAPI-4.0_64bit","*.svc","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","","",strFirstItem);
AddHandler(objHandlers.Collection,"HttpRemotingHandlerFactory-soap-ISAPI-4.0_64bit","*.soap","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"HttpRemotingHandlerFactory-rem-ISAPI-4.0_64bit","*.rem","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"WebServiceHandlerFactory-ISAPI-4.0_64bit","*.asmx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"SimpleHandlerFactory-ISAPI-4.0_64bit","*.ashx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"PageHandlerFactory-ISAPI-4.0_64bit","*.aspx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"AXD-ISAPI-4.0_64bit","*.axd","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","",strFirstItem);
AddHandler(objHandlers.Collection,"svc-ISAPI-4.0_32bit","*.svc","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness32","","","HttpRemotingHandlerFactory-soap-ISAPI-4.0_32bit");
AddHandler(objHandlers.Collection,"rules-ISAPI-4.0_32bit","*.rules","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness32","","","svc-ISAPI-4.0_32bit");
AddHandler(objHandlers.Collection,"xoml-ISAPI-4.0_32bit","*.xoml","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness32","","","rules-ISAPI-4.0_32bit");
AddHandler(objHandlers.Collection,"xamlx-ISAPI-4.0_32bit","*.xamlx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness32","","","xoml-ISAPI-4.0_32bit");
AddHandler(objHandlers.Collection,"aspq-ISAPI-4.0_32bit","*.aspq","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness32","0","","xamlx-ISAPI-4.0_32bit");
AddHandler(objHandlers.Collection,"ScriptResourceIntegrated-4.0","*ScriptResource.axd","GET,HEAD","","System.Web.Handlers.ScriptResourceHandler, System.Web.Extensions, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31BF3856AD364E35","","integratedMode,runtimeVersionv4.0","","","ScriptHandlerFactoryAppServices-Integrated-4.0");
AddHandler(objHandlers.Collection,"TraceHandler-Integrated","trace.axd","GET,HEAD,POST,DEBUG","","System.Web.Handlers.TraceHandler","","integratedMode,runtimeVersionv2.0","","","ISAPI-dll");
AddHandler(objHandlers.Collection,"WebAdminHandler-Integrated","WebAdmin.axd","GET,DEBUG","","System.Web.Handlers.WebAdminHandler","","integratedMode,runtimeVersionv2.0","","","TraceHandler-Integrated");
AddHandler(objHandlers.Collection,"AssemblyResourceLoader-Integrated","WebResource.axd","GET,DEBUG","","System.Web.Handlers.AssemblyResourceLoader","","integratedMode,runtimeVersionv2.0","","","WebAdminHandler-Integrated");
AddHandler(objHandlers.Collection,"PageHandlerFactory-Integrated","*.aspx","GET,HEAD,POST,DEBUG","","System.Web.UI.PageHandlerFactory","","integratedMode,runtimeVersionv2.0","","","AssemblyResourceLoader-Integrated");
AddHandler(objHandlers.Collection,"SimpleHandlerFactory-Integrated","*.ashx","GET,HEAD,POST,DEBUG","","System.Web.UI.SimpleHandlerFactory","","integratedMode,runtimeVersionv2.0","","","PageHandlerFactory-Integrated");
AddHandler(objHandlers.Collection,"svc-ISAPI-2.0-64","*.svc","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","","","HttpRemotingHandlerFactory-soap-ISAPI-2.0");
AddHandler(objHandlers.Collection,"AXD-ISAPI-2.0-64","*.axd","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","0","","svc-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"PageHandlerFactory-ISAPI-2.0-64","*.aspx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","0","","AXD-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"SimpleHandlerFactory-ISAPI-2.0-64","*.ashx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","0","","PageHandlerFactory-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"WebServiceHandlerFactory-ISAPI-2.0-64","*.asmx","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","0","","SimpleHandlerFactory-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"HttpRemotingHandlerFactory-rem-ISAPI-2.0-64","*.rem","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","0","","WebServiceHandlerFactory-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"HttpRemotingHandlerFactory-soap-ISAPI-2.0-64","*.soap","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","0","","HttpRemotingHandlerFactory-rem-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"rules-64-ISAPI-2.0","*.rules","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","","","HttpRemotingHandlerFactory-soap-ISAPI-2.0-64");
AddHandler(objHandlers.Collection,"xoml-64-ISAPI-2.0","*.xoml","*","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll","classicMode,runtimeVersionv2.0,bitness64","","","rules-64-ISAPI-2.0");
AddHandler(objHandlers.Collection,"SSINC-stm","*.stm","GET,HEAD,POST","ServerSideIncludeModule","","","","","File","CGI-exe");
AddHandler(objHandlers.Collection,"SSINC-shtm","*.shtm","GET,HEAD,POST","ServerSideIncludeModule","","","","","File","SSINC-stm");
AddHandler(objHandlers.Collection,"SSINC-shtml","*.shtml","GET,HEAD,POST","ServerSideIncludeModule","","","","","File","SSINC-shtm");
AddHandler(objHandlers.Collection,"ExtensionlessUrlHandler-ISAPI-4.0_64bit","*.","GET,HEAD,POST,DEBUG","IsapiModule","","%windir%\\Microsoft.NET\\Framework64\\v4.0.30319\\aspnet_isapi.dll","classicMode,runtimeVersionv4.0,bitness64","0","","ExtensionlessUrl-ISAPI-4.0_32bit");
AddHandler(objHandlers.Collection,"ExtensionlessUrl-Integrated-4.0","*.","GET,HEAD,POST,DEBUG","","System.Web.Handlers.TransferRequestHandler","","integratedMode,runtimeVersionv4.0","0","","ExtensionlessUrlHandler-ISAPI-4.0_64bit");

// ------------------------------------------------------------
// Commit changes and exit.
// ------------------------------------------------------------

try
{
	objAdminManager.CommitChanges();
}
catch(e)
{
	ErrorMessage(e,"An error occurred trying to commit the changes");
}

WScript.Echo("\nFinished!");
WScript.Quit(0);

// ================================================================================

function AddSection(tmpSectionGroup,tmpSectionName,tmpOverrideModeDefault,tmpAllowDefinition,tmpAllowLocation)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement2(tmpSectionGroup.sections,tmpSectionName);
		var tmpNewSection = null;
		if (tmpElementPosition == -1)
		{
			tmpNewSection = tmpSectionGroup.Sections.AddSection(tmpSectionName);
		}
		else
		{
			tmpNewSection = tmpSectionGroup.Sections.Item(tmpElementPosition);
		}
		// Add the required attributes.
		tmpNewSection.OverrideModeDefault = tmpOverrideModeDefault;
		tmpNewSection.AllowDefinition = tmpAllowDefinition;
		tmpNewSection.AllowLocation = tmpAllowLocation;
	}
	catch(e)
	{
		ErrorMessage(e,"An error occurred trying to add a section");
	}
}

// ================================================================================

function AddGlobalModule(tmpModuleGroup,tmpModuleName,tmpImage,tmpPreCondition,tmpPreviousModuleName)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpModuleGroup,"add",["name",tmpModuleName]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpModuleGroup.DeleteElement(tmpElementPosition);
		// Create a new element
		var tmpNewElement = tmpModuleGroup.CreateNewElement("add");
		// Add the required properties.
		tmpNewElement.Properties.Item("name").Value = tmpModuleName;
		tmpNewElement.Properties.Item("image").Value = tmpImage;
		// Add any optional properties.
		if (tmpPreCondition.length != 0) tmpNewElement.Properties.Item("preCondition").Value = tmpPreCondition;
		// Retrieve the previous index within the collection.
		tmpElementPosition = FindElement3(tmpModuleGroup,tmpPreviousModuleName);
		// Add the new element.
		tmpModuleGroup.AddElement(tmpNewElement, tmpElementPosition + ((tmpElementPosition>0) ? 1 : 0));
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add a global module");
	}
}

// ================================================================================

function AddIsapiFilter(tmpIsapiFilterCollection,tmpName,tmpPath,tmpEnabled,tmpEnableCache,tmpPreCondition,tmpPreviousFilterName)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpIsapiFilterCollection,"filter",["name",tmpName]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpIsapiFilterCollection.DeleteElement(tmpElementPosition);
		// Create a new element
		var tmpNewElement = tmpIsapiFilterCollection.CreateNewElement("filter");
		// Add the required properties.
		tmpNewElement.Properties.Item("name").Value = tmpName;
		tmpNewElement.Properties.Item("path").Value = tmpPath;
		// Add any optional properties.
		if (tmpEnabled.length != 0) tmpNewElement.Properties.Item("enabled").Value = tmpEnabled;
		if (tmpEnableCache.length != 0) tmpNewElement.Properties.Item("enableCache").Value = tmpEnableCache;
		if (tmpPreCondition.length != 0) tmpNewElement.Properties.Item("preCondition").Value = tmpPreCondition;
		// Retrieve the previous index within the collection.
		tmpElementPosition = FindElement3(tmpIsapiFilterCollection,tmpPreviousFilterName);
		// Add the new element.
		tmpIsapiFilterCollection.AddElement(tmpNewElement, tmpElementPosition + ((tmpElementPosition>0) ? 1 : 0));
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add an ISAPI filter");
	}
}

// ================================================================================

function AddIsapiCgiRestriction(tmpIsapiCgiRestrictionCollection,tmpPath,tmpAllowed,tmpGroupId,tmpDescription,tmpPrevious)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpIsapiCgiRestrictionCollection,"add",["path",tmpPath]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpIsapiCgiRestrictionCollection.DeleteElement(tmpElementPosition);
		// Create a new element
		var tmpNewElement = tmpIsapiCgiRestrictionCollection.CreateNewElement("add");
		// Add the required properties.
		tmpNewElement.Properties.Item("path").Value = tmpPath;
		tmpNewElement.Properties.Item("allowed").Value = tmpAllowed;
		// Add any optional properties.
		if (tmpGroupId.length != 0) tmpNewElement.Properties.Item("groupId").Value = tmpGroupId;
		if (tmpDescription.length != 0) tmpNewElement.Properties.Item("description").Value = tmpDescription;
		// Retrieve the previous index within the collection.
		tmpElementPosition = FindElement3(tmpIsapiCgiRestrictionCollection,tmpPrevious);
		// Add the new element.
		tmpIsapiCgiRestrictionCollection.AddElement(tmpNewElement, tmpElementPosition + ((tmpElementPosition>0) ? 1 : 0));
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add an ISAPI/CGI restriction");
	}
}

// ================================================================================

function AddMimeMap(tmpStaticContentCollection,tmpFileExtension,tmpMimeType)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpStaticContentCollection,"mimeMap",["fileExtension",tmpFileExtension]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpStaticContentCollection.DeleteElement(tmpElementPosition);
		// Create a new element
		var tmpNewElement = tmpStaticContentCollection.CreateNewElement("mimeMap");
		// Add the required properties.
		tmpNewElement.Properties.Item("fileExtension").Value = tmpFileExtension;
		tmpNewElement.Properties.Item("mimeType").Value = tmpMimeType;
		// Add the new element.
		tmpStaticContentCollection.AddElement(tmpNewElement, -1);
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add a MIME map");
	}
}

// ================================================================================

function AddTraceProviderDefinitions(tmpTraceProviderDefinitionCollection,tmpParent,tmpName,tmpValue)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition1 = FindElement1(tmpTraceProviderDefinitionCollection,"add",["name",tmpParent]);
		if (tmpElementPosition1 != -1)
		{
			var objWwwServerDefinitions = tmpTraceProviderDefinitionCollection.Item(tmpElementPosition1).ChildElements.Item(0).Collection;
			// Retrieve the index within the collection.
			var tmpElementPosition2 = FindElement1(objWwwServerDefinitions,"add",["name",tmpName]);
			// Delete the item if it already exists.
			if (tmpElementPosition2 != -1) objWwwServerDefinitions.DeleteElement(tmpElementPosition2);
			// Create a new element.
			var tmpNewElement = objWwwServerDefinitions.CreateNewElement("add");
			// Add the required properties.
			tmpNewElement.Properties.Item("name").Value = tmpName;
			tmpNewElement.Properties.Item("value").Value = tmpValue;
			// Add the new element.
			objWwwServerDefinitions.AddElement(tmpNewElement, -1);
		}
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add a trace provider definition");
	}
}

// ================================================================================

function UpdateTraceAreas(tmpTraceAreasCollection,tmpProvider,tmpAreas,tmpVerbosity)
{
	try
	{
		// Retrieve the index within the collection.
		var objTraceAreas = tmpTraceAreasCollection.Item(0).ChildElements.Item(0).Collection;
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(objTraceAreas,"add",["provider",tmpProvider]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) objTraceAreas.DeleteElement(tmpElementPosition);
		// Create a new element.
		var tmpNewElement = objTraceAreas.CreateNewElement("add");
		// Add the required properties.
		tmpNewElement.Properties.Item("provider").Value = tmpProvider;
		tmpNewElement.Properties.Item("areas").Value = tmpAreas;
		tmpNewElement.Properties.Item("verbosity").Value = tmpVerbosity;
		// Add the new element.
		objTraceAreas.AddElement(tmpNewElement, -1);
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to update the trace areas");
	}
}

// ================================================================================

function UpdateWebDavGlobalSettings(tmpWebDavStore,tmpName,tmpImage,tmpImage32)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpWebDavStore,"add",["name",tmpName]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpWebDavStore.DeleteElement(tmpElementPosition);
		// Create a new element.
		var tmpNewElement = tmpWebDavStore.CreateNewElement("add")
		// Add the required properties.
		tmpNewElement.Properties.Item("name").Value = tmpName;
		tmpNewElement.Properties.Item("image").Value = tmpImage;
		tmpNewElement.Properties.Item("image32").Value = tmpImage32;
		// Add the new element.
		tmpWebDavStore.AddElement(tmpNewElement, -1);
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to update the WebDAV settings");
	}
}

// ================================================================================

function AddModule(tmpModuleGroup,tmpModuleName,tmpLockItem,tmpType,tmpPreCondition,tmpPreviousModuleName)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpModuleGroup,"add",["name",tmpModuleName]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpModuleGroup.DeleteElement(tmpElementPosition);
		// Create a new element.
		var tmpNewElement = tmpModuleGroup.CreateNewElement("add");
		// Add the required properties.
		tmpNewElement.Properties.Item("name").Value = tmpModuleName;
		// Add any optional properties.
		if (tmpLockItem.length != 0) tmpNewElement.SetMetadata("lockItem", (tmpLockItem.toLowerCase() == "true") ? true : false );
		if (tmpType.length != 0) tmpNewElement.Properties.Item("type").Value = tmpType;
		if (tmpPreCondition.length != 0) tmpNewElement.Properties.Item("preCondition").Value = tmpPreCondition;
		// Retrieve the previous index within the collection.
		tmpElementPosition = FindElement3(tmpModuleGroup,tmpPreviousModuleName);
		// Add the new element.
		tmpModuleGroup.AddElement(tmpNewElement, tmpElementPosition + ((tmpElementPosition>0) ? 1 : 0));
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add a module");
	}
}

// ================================================================================

function AddHandler(tmpHandlerCollection,tmpName,tmpPath,tmpVerb,tmpModules,tmpType,tmpScriptProcessor,tmpPreCondition,tmpPesponseBufferLimit,tmpResourceType,tmpPrevious)
{
	try
	{
		// Retrieve the index within the collection.
		var tmpElementPosition = FindElement1(tmpHandlerCollection,"add",["name",tmpName]);
		// Delete the item if it already exists.
		if (tmpElementPosition != -1) tmpHandlerCollection.DeleteElement(tmpElementPosition);
		// Create a new element.
		var tmpNewElement = tmpHandlerCollection.CreateNewElement("add");
		// Add the required properties.
		tmpNewElement.Properties.Item("name").Value = tmpName;
		tmpNewElement.Properties.Item("verb").Value = tmpVerb;
		tmpNewElement.Properties.Item("path").Value = tmpPath;
		// Add any optional properties.
		if (tmpType.length != 0) tmpNewElement.Properties.Item("type").Value = tmpType;
		if (tmpModules.length != 0) tmpNewElement.Properties.Item("modules").Value = tmpModules;
		if (tmpScriptProcessor.length != 0) tmpNewElement.Properties.Item("scriptProcessor").Value = tmpScriptProcessor;
		if (tmpPreCondition.length != 0) tmpNewElement.Properties.Item("preCondition").Value = tmpPreCondition;
		if (tmpPesponseBufferLimit.length != 0) tmpNewElement.Properties.Item("responseBufferLimit").Value = tmpPesponseBufferLimit;
		if (tmpResourceType.length != 0) tmpNewElement.Properties.Item("resourceType").Value = tmpResourceType;
		// Retrieve the previous index within the collection.
		tmpElementPosition = FindElement3(tmpHandlerCollection,tmpPrevious);
		// Add the new element.
		tmpHandlerCollection.AddElement(tmpNewElement, tmpElementPosition + ((tmpElementPosition>0) ? 1 : 0));
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to add a handler");
	}
}

// ================================================================================

function PadNumber(tmpNumber)
{
	return (tmpNumber < 10) ? ("0" + tmpNumber.toString()) : tmpNumber.toString();
}

// ================================================================================

function ErrorMessage(tmpError,tmpMessage)
{
	WScript.Echo("\n" + tmpMessage + ":\n" + tmpError.description);
	WScript.Quit(tmpError.number);
}

// ================================================================================

function GetAdminManager()
{
	try
	{
		var tmpVersionManager = WScript.CreateObject("Microsoft.IIS.VersionManager");
		var tmpVersionObject = tmpVersionManager.GetVersionObject("10.0", 1);
		var tmpAdminManager = tmpVersionObject.CreateObjectFromProgId("Microsoft.ApplicationHost.WritableAdminManager");
		return tmpAdminManager;
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to obtain the Admin Manager");
	}
}

// ================================================================================

function GetUserDirectory()
{
	try
	{
		var tmpVersionManager = WScript.CreateObject("Microsoft.IIS.VersionManager");
		var tmpVersionObject = tmpVersionManager.GetVersionObject("10.0", 1);
		var tmpUserData = tmpVersionObject.GetPropertyValue("userInstanceHelper")
		var tmpUserDirectory = tmpUserData.IISDirectory;
		if (tmpUserDirectory.length > 0) return tmpUserDirectory;
		throw("The User Directory cannot be determined.");
	}
	catch(e)
	{
		ErrorMessage(e,"The following error occurred trying to obtain the User Directory");
	}
}

// ================================================================================

function FindSectionGroup(tmpParentSectionGroup,tmpName)
{
	try
	{
		// Retrieve the index within the sectionGroup.
		var tmpElementPosition = FindElement2(tmpParentSectionGroup,tmpName);
		// Fail completely if we can't retrive the index.
		if (tmpElementPosition == -1) throw("Cannot retrieve index for '" & tmpName & "'.");
		return tmpParentSectionGroup.Item(tmpElementPosition);
	}
	catch(e)
	{
		ErrorMessage(e,"An error occurred trying to add a section group");
	}
}

// ================================================================================

function FindElement1(tmpCollection, tmpElementTagName, tmpValuesArray)
{
   for (var tmpCount1 = 0; tmpCount1 < tmpCollection.Count; ++tmpCount1)
   {
      var tmpElement = tmpCollection.Item(tmpCount1);
      if (tmpElement.Name == tmpElementTagName)
      {
         var tmpMatches = true;
         for (var tmpCount2 = 0; tmpCount2 < tmpValuesArray.length; tmpCount2 += 2)
         {
            var tmpProperty = tmpElement.GetPropertyByName(tmpValuesArray[tmpCount2]);
            var tmpValue = tmpProperty.Value;
            if (tmpValue != null) tmpValue = tmpValue.toString();
            if (tmpValue != tmpValuesArray[tmpCount2 + 1])
            {
               tmpMatches = false;
               break;
            }
         }
         if (tmpMatches) return tmpCount1;
      }
   }
   return -1;
}

// ================================================================================

function FindElement2(tmpCollection,tmpName)
{
	for (var tmpCount = 0; tmpCount < tmpCollection.Count; ++tmpCount)
	{
		var tmpElement = tmpCollection.Item(tmpCount);
		if (tmpElement.Name == tmpName)
		{
			return tmpCount;
		}
	}
   return -1;
}

// ================================================================================

function FindElement3(tmpCollection,tmpName)
{
	if ((tmpName.length ==0) || (tmpName.toLowerCase() == strLastItem.toLowerCase())) return -1;
	if ((tmpName.length ==0) || (tmpName.toLowerCase() == strFirstItem.toLowerCase())) return 0;	
	return FindElement1(tmpCollection,"add",["name",tmpName]);
}

// SIG // Begin signature block
// SIG // MIIdggYJKoZIhvcNAQcCoIIdczCCHW8CAQExCzAJBgUr
// SIG // DgMCGgUAMGcGCisGAQQBgjcCAQSgWTBXMDIGCisGAQQB
// SIG // gjcCAR4wJAIBAQQQEODJBs441BGiowAQS9NQkAIBAAIB
// SIG // AAIBAAIBAAIBADAhMAkGBSsOAwIaBQAEFJC0/xuhb8l1
// SIG // /xeUfXYcZF/+spGEoIIYYjCCBMMwggOroAMCAQICEzMA
// SIG // AACb4HQ3yz1NjS4AAAAAAJswDQYJKoZIhvcNAQEFBQAw
// SIG // dzELMAkGA1UEBhMCVVMxEzARBgNVBAgTCldhc2hpbmd0
// SIG // b24xEDAOBgNVBAcTB1JlZG1vbmQxHjAcBgNVBAoTFU1p
// SIG // Y3Jvc29mdCBDb3Jwb3JhdGlvbjEhMB8GA1UEAxMYTWlj
// SIG // cm9zb2Z0IFRpbWUtU3RhbXAgUENBMB4XDTE2MDMzMDE5
// SIG // MjEyOVoXDTE3MDYzMDE5MjEyOVowgbMxCzAJBgNVBAYT
// SIG // AlVTMRMwEQYDVQQIEwpXYXNoaW5ndG9uMRAwDgYDVQQH
// SIG // EwdSZWRtb25kMR4wHAYDVQQKExVNaWNyb3NvZnQgQ29y
// SIG // cG9yYXRpb24xDTALBgNVBAsTBE1PUFIxJzAlBgNVBAsT
// SIG // Hm5DaXBoZXIgRFNFIEVTTjo3MjhELUM0NUYtRjlFQjEl
// SIG // MCMGA1UEAxMcTWljcm9zb2Z0IFRpbWUtU3RhbXAgU2Vy
// SIG // dmljZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoC
// SIG // ggEBAI2j4s+Bi9fLvwOiYPY7beLUGLA3BdWNNpwOc85N
// SIG // f6IQsnxDeywYV7ysp6aGfXmhtd4yZvmO/CDNq3N3z3ed
// SIG // b2Cca3jzxa2pvVtMK1WqUoBBQ0FmmaXwMGiGug8hch/D
// SIG // dT+SdsEA15ksqFk/wWKRbQn2ztMiui0An2bLU9HKVjpY
// SIG // TCGyhaOYZYzHiUpFWHurU0CfjGqyBcX+HuL/CqGootvL
// SIG // IY18lTDeMReKDelfzEJwyqQVFG6ED8LC/WwCTJOxTLbO
// SIG // tuzitc2aGhD1SOVXEHfqgd1fhEIycETJyryw+/dIOdhg
// SIG // dUmts79odC6UDhy+wXBydBAOzNtrUB8x6jT6bD0CAwEA
// SIG // AaOCAQkwggEFMB0GA1UdDgQWBBSWlbGeE1O6WCFGNOJ8
// SIG // xzlKbCDwdzAfBgNVHSMEGDAWgBQjNPjZUkZwCu1A+3b7
// SIG // syuwwzWzDzBUBgNVHR8ETTBLMEmgR6BFhkNodHRwOi8v
// SIG // Y3JsLm1pY3Jvc29mdC5jb20vcGtpL2NybC9wcm9kdWN0
// SIG // cy9NaWNyb3NvZnRUaW1lU3RhbXBQQ0EuY3JsMFgGCCsG
// SIG // AQUFBwEBBEwwSjBIBggrBgEFBQcwAoY8aHR0cDovL3d3
// SIG // dy5taWNyb3NvZnQuY29tL3BraS9jZXJ0cy9NaWNyb3Nv
// SIG // ZnRUaW1lU3RhbXBQQ0EuY3J0MBMGA1UdJQQMMAoGCCsG
// SIG // AQUFBwMIMA0GCSqGSIb3DQEBBQUAA4IBAQAhHbNT6TtG
// SIG // gaH6KhPjWiAkunalO7Z3yJFyBNbq/tKbIi+TCKKwbu8C
// SIG // pblWXv1l9o0Sfeon3j+guC4zMteWWj/DdDnJD6m2utr+
// SIG // EGjPiP2PIN6ysdZdKJMnt8IHpEclZbtS1XFNKWnoC1DH
// SIG // jJWWoF6sNzkC1V7zVCh5cdsXw0P8zWor+Q85QER8LGjI
// SIG // 0oHomSKrIFbm5O8khptmVk474u64ZPfln8p1Cu58lp9Z
// SIG // 4aygt9ZpvUIm0vWlh1IB7Cl++wW05tiXfBOAcTVfkybn
// SIG // 5F90lXF8A421H3X1orZhPe7EbIleZAR/KUts1EjqSkpM
// SIG // 54JutTq/VyYRyHiA1YDNDrtkMIIGBzCCA++gAwIBAgIK
// SIG // YRZoNAAAAAAAHDANBgkqhkiG9w0BAQUFADBfMRMwEQYK
// SIG // CZImiZPyLGQBGRYDY29tMRkwFwYKCZImiZPyLGQBGRYJ
// SIG // bWljcm9zb2Z0MS0wKwYDVQQDEyRNaWNyb3NvZnQgUm9v
// SIG // dCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkwHhcNMDcwNDAz
// SIG // MTI1MzA5WhcNMjEwNDAzMTMwMzA5WjB3MQswCQYDVQQG
// SIG // EwJVUzETMBEGA1UECBMKV2FzaGluZ3RvbjEQMA4GA1UE
// SIG // BxMHUmVkbW9uZDEeMBwGA1UEChMVTWljcm9zb2Z0IENv
// SIG // cnBvcmF0aW9uMSEwHwYDVQQDExhNaWNyb3NvZnQgVGlt
// SIG // ZS1TdGFtcCBQQ0EwggEiMA0GCSqGSIb3DQEBAQUAA4IB
// SIG // DwAwggEKAoIBAQCfoWyx39tIkip8ay4Z4b3i48WZUSNQ
// SIG // rc7dGE4kD+7Rp9FMrXQwIBHrB9VUlRVJlBtCkq6YXDAm
// SIG // 2gBr6Hu97IkHD/cOBJjwicwfyzMkh53y9GccLPx754gd
// SIG // 6udOo6HBI1PKjfpFzwnQXq/QsEIEovmmbJNn1yjcRlOw
// SIG // htDlKEYuJ6yGT1VSDOQDLPtqkJAwbofzWTCd+n7Wl7Po
// SIG // IZd++NIT8wi3U21StEWQn0gASkdmEScpZqiX5NMGgUqi
// SIG // +YSnEUcUCYKfhO1VeP4Bmh1QCIUAEDBG7bfeI0a7xC1U
// SIG // n68eeEExd8yb3zuDk6FhArUdDbH895uyAc4iS1T/+QXD
// SIG // wiALAgMBAAGjggGrMIIBpzAPBgNVHRMBAf8EBTADAQH/
// SIG // MB0GA1UdDgQWBBQjNPjZUkZwCu1A+3b7syuwwzWzDzAL
// SIG // BgNVHQ8EBAMCAYYwEAYJKwYBBAGCNxUBBAMCAQAwgZgG
// SIG // A1UdIwSBkDCBjYAUDqyCYEBWJ5flJRP8KuEKU5VZ5KSh
// SIG // Y6RhMF8xEzARBgoJkiaJk/IsZAEZFgNjb20xGTAXBgoJ
// SIG // kiaJk/IsZAEZFgltaWNyb3NvZnQxLTArBgNVBAMTJE1p
// SIG // Y3Jvc29mdCBSb290IENlcnRpZmljYXRlIEF1dGhvcml0
// SIG // eYIQea0WoUqgpa1Mc1j0BxMuZTBQBgNVHR8ESTBHMEWg
// SIG // Q6BBhj9odHRwOi8vY3JsLm1pY3Jvc29mdC5jb20vcGtp
// SIG // L2NybC9wcm9kdWN0cy9taWNyb3NvZnRyb290Y2VydC5j
// SIG // cmwwVAYIKwYBBQUHAQEESDBGMEQGCCsGAQUFBzAChjho
// SIG // dHRwOi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpL2NlcnRz
// SIG // L01pY3Jvc29mdFJvb3RDZXJ0LmNydDATBgNVHSUEDDAK
// SIG // BggrBgEFBQcDCDANBgkqhkiG9w0BAQUFAAOCAgEAEJeK
// SIG // w1wDRDbd6bStd9vOeVFNAbEudHFbbQwTq86+e4+4LtQS
// SIG // ooxtYrhXAstOIBNQmd16QOJXu69YmhzhHQGGrLt48ovQ
// SIG // 7DsB7uK+jwoFyI1I4vBTFd1Pq5Lk541q1YDB5pTyBi+F
// SIG // A+mRKiQicPv2/OR4mS4N9wficLwYTp2OawpylbihOZxn
// SIG // LcVRDupiXD8WmIsgP+IHGjL5zDFKdjE9K3ILyOpwPf+F
// SIG // ChPfwgphjvDXuBfrTot/xTUrXqO/67x9C0J71FNyIe4w
// SIG // yrt4ZVxbARcKFA7S2hSY9Ty5ZlizLS/n+YWGzFFW6J1w
// SIG // lGysOUzU9nm/qhh6YinvopspNAZ3GmLJPR5tH4LwC8cs
// SIG // u89Ds+X57H2146SodDW4TsVxIxImdgs8UoxxWkZDFLyz
// SIG // s7BNZ8ifQv+AeSGAnhUwZuhCEl4ayJ4iIdBD6Svpu/RI
// SIG // zCzU2DKATCYqSCRfWupW76bemZ3KOm+9gSd0BhHudiG/
// SIG // m4LBJ1S2sWo9iaF2YbRuoROmv6pH8BJv/YoybLL+31HI
// SIG // jCPJZr2dHYcSZAI9La9Zj7jkIeW1sMpjtHhUBdRBLlCs
// SIG // lLCleKuzoJZ1GtmShxN1Ii8yqAhuoFuMJb+g74TKIdbr
// SIG // Hk/Jmu5J4PcBZW+JC33Iacjmbuqnl84xKf8OxVtc2E0b
// SIG // odj6L54/LlUWa8kTo/0wggYOMIID9qADAgECAhMzAAAA
// SIG // Y4j+Hjj4cronAAAAAABjMA0GCSqGSIb3DQEBCwUAMH4x
// SIG // CzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpXYXNoaW5ndG9u
// SIG // MRAwDgYDVQQHEwdSZWRtb25kMR4wHAYDVQQKExVNaWNy
// SIG // b3NvZnQgQ29ycG9yYXRpb24xKDAmBgNVBAMTH01pY3Jv
// SIG // c29mdCBDb2RlIFNpZ25pbmcgUENBIDIwMTEwHhcNMTUx
// SIG // MDI4MjAzMTQ1WhcNMTcwMTI4MjAzMTQ1WjCBgjELMAkG
// SIG // A1UEBhMCVVMxEzARBgNVBAgTCldhc2hpbmd0b24xEDAO
// SIG // BgNVBAcTB1JlZG1vbmQxHjAcBgNVBAoTFU1pY3Jvc29m
// SIG // dCBDb3Jwb3JhdGlvbjEMMAoGA1UECxMDQU9DMR4wHAYD
// SIG // VQQDExVNaWNyb3NvZnQgQ29ycG9yYXRpb24wggEiMA0G
// SIG // CSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDHLephCyTz
// SIG // Fi2LM11fBQZxMvQtqrdzfJW3ik1hCfhDoravcKjRFC+Q
// SIG // 8vWy6Y1/5RGtJWQAeh/IbEVGGMDC/WkZ5mpM0Em2MMdb
// SIG // g+DJ+e6LFkzQIwze69YSRQ4EkNsATq/d1fHe3rcKzaoo
// SIG // tgDCj3NKTrex1pawEmwvw4GOTR8aE5L86ht0498wUnXd
// SIG // 7Szj6Dm+P+NmJh0D5EtBk6TgBx7v1HSGWK0au37eZQg7
// SIG // TJTnsPMzG8sxWWSXTDfB49pUYIEaRzMaP0uP238sS415
// SIG // hqcZMxnCGfybFk4/GqHipS8gI3iodR81GEDz/QhVGaxx
// SIG // vS4YeAujdAOSoJovzSTkOBSRAgMBAAGjggF+MIIBejAf
// SIG // BgNVHSUEGDAWBggrBgEFBQcDAwYKKwYBBAGCN0wIATAd
// SIG // BgNVHQ4EFgQUUIn2BTP3tJJaeNpk2oQQ12k24QwwUAYD
// SIG // VR0RBEkwR6RFMEMxDDAKBgNVBAsTA0FPQzEzMDEGA1UE
// SIG // BRMqMzE2NDIrNDllOGMzZjMtMjM1OS00N2Y2LWEzYmUt
// SIG // NmM4YzQ3NTFjNGI2MB8GA1UdIwQYMBaAFEhuZOVQBdOC
// SIG // qhc3NyK1bajKdQKVMFQGA1UdHwRNMEswSaBHoEWGQ2h0
// SIG // dHA6Ly93d3cubWljcm9zb2Z0LmNvbS9wa2lvcHMvY3Js
// SIG // L01pY0NvZFNpZ1BDQTIwMTFfMjAxMS0wNy0wOC5jcmww
// SIG // YQYIKwYBBQUHAQEEVTBTMFEGCCsGAQUFBzAChkVodHRw
// SIG // Oi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2NlcnRz
// SIG // L01pY0NvZFNpZ1BDQTIwMTFfMjAxMS0wNy0wOC5jcnQw
// SIG // DAYDVR0TAQH/BAIwADANBgkqhkiG9w0BAQsFAAOCAgEA
// SIG // gY8GUdukQMDfXYNRnwm5hRPX9C/pYJda1gc5Ngsu+ks8
// SIG // 2KKFr++m0VwU6QtHn65P3mVaJqTcvmFGGlrbJdl6kOeF
// SIG // ptIlr+msHHK0Xw21Rn0uj7RMnGnkVN7TIdiIe28UEdhg
// SIG // W8NRkPE7j06gEbhd712GkkWsp9MSb1AgWZNVjYvZf0o+
// SIG // 4fVk2jnI3nuoinlZ+DLiJ4qb4apDcQ97wQLRXPTCqy/m
// SIG // 9Xbn2kvMFj3AvTUveVk/U+QFhZnfV6iGBwbd1Y9lru9b
// SIG // aYiY0G54crle6jYWA/0FRFdHuHqyKxxKUM1Om5/fKv57
// SIG // vHT8e4KeX6wq3Q/di0HzDcNCcI7RLnOaeTzEsHG/5mLy
// SIG // Ba5uE/OA6e0waGZ0muBY3t/AQxd4JalaMw8m3fohlOZ5
// SIG // qBCQLUpYsRFwpFKM8Vs5nAE5N8xk7fWktZ35UTg3SgJE
// SIG // 7793lv64vy4uLbimMq29vryJtF5z/9f7sZP4KnGvSI2g
// SIG // oXNhz+i/vxNbOWP/8wjjMNu2oq7/ZCTEAMMiUqau2k+3
// SIG // qI8N1V0Ouijq4s1dsw7L5UOugyoKOL6StDQDysFmvEQA
// SIG // 1gwvz7S1JoXyJXUV1yevF4jwAxY/HquZ0iDWxrmzd6Hj
// SIG // d1jHAv9C2DsoMSp1JFwmdaO2Th5qiZA21qnTPIcTXZR6
// SIG // L+ZanAcr3KQJswa+NnI4eaQwggd6MIIFYqADAgECAgph
// SIG // DpDSAAAAAAADMA0GCSqGSIb3DQEBCwUAMIGIMQswCQYD
// SIG // VQQGEwJVUzETMBEGA1UECBMKV2FzaGluZ3RvbjEQMA4G
// SIG // A1UEBxMHUmVkbW9uZDEeMBwGA1UEChMVTWljcm9zb2Z0
// SIG // IENvcnBvcmF0aW9uMTIwMAYDVQQDEylNaWNyb3NvZnQg
// SIG // Um9vdCBDZXJ0aWZpY2F0ZSBBdXRob3JpdHkgMjAxMTAe
// SIG // Fw0xMTA3MDgyMDU5MDlaFw0yNjA3MDgyMTA5MDlaMH4x
// SIG // CzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpXYXNoaW5ndG9u
// SIG // MRAwDgYDVQQHEwdSZWRtb25kMR4wHAYDVQQKExVNaWNy
// SIG // b3NvZnQgQ29ycG9yYXRpb24xKDAmBgNVBAMTH01pY3Jv
// SIG // c29mdCBDb2RlIFNpZ25pbmcgUENBIDIwMTEwggIiMA0G
// SIG // CSqGSIb3DQEBAQUAA4ICDwAwggIKAoICAQCr8PpyEBwu
// SIG // rdhuqoIQTTS68rZYIZ9CGypr6VpQqrgGOBoESbp/wwwe
// SIG // 3TdrxhLYC/A4wpkGsMg51QEUMULTiQ15ZId+lGAkbK+e
// SIG // SZzpaF7S35tTsgosw6/ZqSuuegmv15ZZymAaBelmdugy
// SIG // UiYSL+erCFDPs0S3XdjELgN1q2jzy23zOlyhFvRGuuA4
// SIG // ZKxuZDV4pqBjDy3TQJP4494HDdVceaVJKecNvqATd76U
// SIG // Pe/74ytaEB9NViiienLgEjq3SV7Y7e1DkYPZe7J7hhvZ
// SIG // PrGMXeiJT4Qa8qEvWeSQOy2uM1jFtz7+MtOzAz2xsq+S
// SIG // OH7SnYAs9U5WkSE1JcM5bmR/U7qcD60ZI4TL9LoDho33
// SIG // X/DQUr+MlIe8wCF0JV8YKLbMJyg4JZg5SjbPfLGSrhwj
// SIG // p6lm7GEfauEoSZ1fiOIlXdMhSz5SxLVXPyQD8NF6Wy/V
// SIG // I+NwXQ9RRnez+ADhvKwCgl/bwBWzvRvUVUvnOaEP6SNJ
// SIG // vBi4RHxF5MHDcnrgcuck379GmcXvwhxX24ON7E1JMKer
// SIG // jt/sW5+v/N2wZuLBl4F77dbtS+dJKacTKKanfWeA5opi
// SIG // eF+yL4TXV5xcv3coKPHtbcMojyyPQDdPweGFRInECUzF
// SIG // 1KVDL3SV9274eCBYLBNdYJWaPk8zhNqwiBfenk70lrC8
// SIG // RqBsmNLg1oiMCwIDAQABo4IB7TCCAekwEAYJKwYBBAGC
// SIG // NxUBBAMCAQAwHQYDVR0OBBYEFEhuZOVQBdOCqhc3NyK1
// SIG // bajKdQKVMBkGCSsGAQQBgjcUAgQMHgoAUwB1AGIAQwBB
// SIG // MAsGA1UdDwQEAwIBhjAPBgNVHRMBAf8EBTADAQH/MB8G
// SIG // A1UdIwQYMBaAFHItOgIxkEO5FAVO4eqnxzHRI4k0MFoG
// SIG // A1UdHwRTMFEwT6BNoEuGSWh0dHA6Ly9jcmwubWljcm9z
// SIG // b2Z0LmNvbS9wa2kvY3JsL3Byb2R1Y3RzL01pY1Jvb0Nl
// SIG // ckF1dDIwMTFfMjAxMV8wM18yMi5jcmwwXgYIKwYBBQUH
// SIG // AQEEUjBQME4GCCsGAQUFBzAChkJodHRwOi8vd3d3Lm1p
// SIG // Y3Jvc29mdC5jb20vcGtpL2NlcnRzL01pY1Jvb0NlckF1
// SIG // dDIwMTFfMjAxMV8wM18yMi5jcnQwgZ8GA1UdIASBlzCB
// SIG // lDCBkQYJKwYBBAGCNy4DMIGDMD8GCCsGAQUFBwIBFjNo
// SIG // dHRwOi8vd3d3Lm1pY3Jvc29mdC5jb20vcGtpb3BzL2Rv
// SIG // Y3MvcHJpbWFyeWNwcy5odG0wQAYIKwYBBQUHAgIwNB4y
// SIG // IB0ATABlAGcAYQBsAF8AcABvAGwAaQBjAHkAXwBzAHQA
// SIG // YQB0AGUAbQBlAG4AdAAuIB0wDQYJKoZIhvcNAQELBQAD
// SIG // ggIBAGfyhqWY4FR5Gi7T2HRnIpsLlhHhY5KZQpZ90nkM
// SIG // kMFlXy4sPvjDctFtg/6+P+gKyju/R6mj82nbY78iNaWX
// SIG // XWWEkH2LRlBV2AySfNIaSxzzPEKLUtCw/WvjPgcuKZvm
// SIG // PRul1LUdd5Q54ulkyUQ9eHoj8xN9ppB0g430yyYCRirC
// SIG // ihC7pKkFDJvtaPpoLpWgKj8qa1hJYx8JaW5amJbkg/TA
// SIG // j/NGK978O9C9Ne9uJa7lryft0N3zDq+ZKJeYTQ49C/II
// SIG // idYfwzIY4vDFLc5bnrRJOQrGCsLGra7lstnbFYhRRVg4
// SIG // MnEnGn+x9Cf43iw6IGmYslmJaG5vp7d0w0AFBqYBKig+
// SIG // gj8TTWYLwLNN9eGPfxxvFX1Fp3blQCplo8NdUmKGwx1j
// SIG // NpeG39rz+PIWoZon4c2ll9DuXWNB41sHnIc+BncG0Qax
// SIG // dR8UvmFhtfDcxhsEvt9Bxw4o7t5lL+yX9qFcltgA1qFG
// SIG // vVnzl6UJS0gQmYAf0AApxbGbpT9Fdx41xtKiop96eiL6
// SIG // SJUfq/tHI4D1nvi/a7dLl+LrdXga7Oo3mXkYS//WsyNo
// SIG // deav+vyL6wuA6mk7r/ww7QRMjt/fdW1jkT3RnVZOT7+A
// SIG // VyKheBEyIXrvQQqxP/uozKRdwaGIm1dxVk5IRcBCyZt2
// SIG // WwqASGv9eZ/BvW1taslScxMNelDNMYIEjDCCBIgCAQEw
// SIG // gZUwfjELMAkGA1UEBhMCVVMxEzARBgNVBAgTCldhc2hp
// SIG // bmd0b24xEDAOBgNVBAcTB1JlZG1vbmQxHjAcBgNVBAoT
// SIG // FU1pY3Jvc29mdCBDb3Jwb3JhdGlvbjEoMCYGA1UEAxMf
// SIG // TWljcm9zb2Z0IENvZGUgU2lnbmluZyBQQ0EgMjAxMQIT
// SIG // MwAAAGOI/h44+HK6JwAAAAAAYzAJBgUrDgMCGgUAoIGg
// SIG // MBkGCSqGSIb3DQEJAzEMBgorBgEEAYI3AgEEMBwGCisG
// SIG // AQQBgjcCAQsxDjAMBgorBgEEAYI3AgEVMCMGCSqGSIb3
// SIG // DQEJBDEWBBTlNIw17tmhVXVaWUfscHjrkjWj2zBABgor
// SIG // BgEEAYI3AgEMMTIwMKAYgBYASQBJAFMAIABFAHgAcABy
// SIG // AGUAcwBzoRSAEmh0dHA6Ly93d3cuaWlzLm5ldDANBgkq
// SIG // hkiG9w0BAQEFAASCAQBLhyr4k7C+9+HlDbHoxtbS/Typ
// SIG // OoYsKQnlxec4R3UoxFSbyVRnT0VFUUbpFa28xvYiAJkv
// SIG // QBjHrNxe+jNOJsNPsxgzSuYPzoIqHOTnmbmEK5IB1qKJ
// SIG // F0WLPBnukyhZFQjPPlvArbIfQhmsnaqpJwkJ1EzMv9bi
// SIG // fHeOX0HLNmuXScnaD1OM6JILg3YPk4fx63aTTEZ3kWT4
// SIG // V5FuKQ3JNOK+nXpWkJ3EvkxGJgZzfZ7hNEm/G+NHkhmS
// SIG // RJ0z2o6Lntpan0+oLRQjwTwh9Pz8YQ22ZUjB7EeEbD5X
// SIG // t/5StKM30FATe0xXODCpUl5nS2mKgQVvtfDYowLTtVOu
// SIG // Q9FdYpMcoYICKDCCAiQGCSqGSIb3DQEJBjGCAhUwggIR
// SIG // AgEBMIGOMHcxCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpX
// SIG // YXNoaW5ndG9uMRAwDgYDVQQHEwdSZWRtb25kMR4wHAYD
// SIG // VQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xITAfBgNV
// SIG // BAMTGE1pY3Jvc29mdCBUaW1lLVN0YW1wIFBDQQITMwAA
// SIG // AJvgdDfLPU2NLgAAAAAAmzAJBgUrDgMCGgUAoF0wGAYJ
// SIG // KoZIhvcNAQkDMQsGCSqGSIb3DQEHATAcBgkqhkiG9w0B
// SIG // CQUxDxcNMTYwNjAzMjM0OTI1WjAjBgkqhkiG9w0BCQQx
// SIG // FgQUhguIvBNYFZ2czKI5tt3/4GZRVf4wDQYJKoZIhvcN
// SIG // AQEFBQAEggEAdhreVZYFmo7RDchpt9MpjMX6B4SSf6Wq
// SIG // dFAvESwr7sDrtwYY6TygKuX02Dj7MChWqHlNn3BNRadj
// SIG // yhd9GF2XExOF/IKByxrxN0UEA/90t96oY6EwjEKTN/mt
// SIG // tbFApK4J+FG+YgmV/MiZ66i2sz4JjuTfziU0aKKWIqvp
// SIG // TQcrnLQmi0Z5R4Yi4NKkLDw6eUqzcrnZwgcUDM4A9Bt7
// SIG // xvbcVIIRjanpUpU49Lmqodp2Q6Is3QLs+MRBosEIN0h1
// SIG // TfjtaycO+VUkdPal5hk/CWTUAU8iCPAgj2dZ3u+YjO9s
// SIG // DZv59Qa+MWoUeu81V30OReBowkD1X2Qa4iAjHWcFLRJROg==
// SIG // End signature block
