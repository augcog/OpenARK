// Configures Windows Communication Foundation 3.5 for WebMatrix
// 
// Usage: 
//    WCF35Setup.js [un]install

function WCFHandler(name, path, mode, bitness)
{
    this._name = name;
    this._path = path;
    this._mode = mode;
    this._bitness = bitness;
}

var moduleName = "ServiceModel";

var wcfHandlers = new Array( new WCFHandler("svc-Integrated", "*.svc", "integrated", null),
                             new WCFHandler("rules-Integrated", "*.rules", "integrated", null),
                             new WCFHandler("xoml-Integrated", "*.xoml", "integrated", null),

                             new WCFHandler("svc-ISAPI-2.0", "*.svc", "classic", "x86"),
                             new WCFHandler("rules-ISAPI-2.0", "*.rules", "classic", "x86"),
                             new WCFHandler("xoml-ISAPI-2.0", "*.xoml", "classic", "x86"),

                             new WCFHandler("svc-ISAPI-2.0-64", "*.svc", "classic", "x64"),
                             new WCFHandler("rules-64-ISAPI-2.0", "*.rules", "classic", "x64"),
                             new WCFHandler("xoml-64-ISAPI-2.0", "*.xoml", "classic", "x64"));

//
// main start
//
try { 
    var mode = ParseArguments();
    if (mode == "install")
    {
        UninstallWCF();
        InstallWCF(); 
        WScript.Echo("WCF 3.5 has been configured for IIS Express.");
    }
    else if (mode == "uninstall")
    {
        UninstallWCF();
        WScript.Echo("WCF 3.5 has been uninstalled from IIS Express.");
    }
    else
    {
        PrintUsage();
    }
} 
catch(e) { 
    WScript.Echo("An error occurred:\r\n " + e.description); 
} 
//
// main end
//

function InstallWCF() { 
    var adminManager = GetAdminManager(); 
 
    AddModule(adminManager);
    AddHandlers(adminManager);
 
    adminManager.CommitChanges(); 
} 

function UninstallWCF() {
    var adminManager = GetAdminManager(); 
    var moduleSection = adminManager.GetAdminSection("system.webServer/modules", "MACHINE/WEBROOT/APPHOST");

    var modulePosition = FindElement(moduleSection.Collection, "add", ["name", moduleName]); 
    if (modulePosition != -1) 
    {
      moduleSection.Collection.DeleteElement(modulePosition); 
    }

    var handlerSection = adminManager.GetAdminSection("system.webServer/handlers", "MACHINE/WEBROOT/APPHOST");

    for (i = 0; i < wcfHandlers.length; i++)
    {
        var svcPosition = FindElement(handlerSection.Collection, "add", ["name", wcfHandlers[i]._name]); 
        if (svcPosition != -1) 
        {
          handlerSection.Collection.DeleteElement(svcPosition); 
        }
    }
    
    adminManager.CommitChanges(); 
}

function AddModule(adminManager)
{
    var moduleSection = adminManager.GetAdminSection("system.webServer/modules", "MACHINE/WEBROOT/APPHOST");
    var element = moduleSection.Collection.CreateNewElement("add"); 

    element.Properties.Item("name").Value = moduleName; 
    element.Properties.Item("type").Value = "System.ServiceModel.Activation.HttpModule, System.ServiceModel, Version=3.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089";
    element.Properties.Item("preCondition").Value = "managedHandler,runtimeVersionv2.0";

    moduleSection.Collection.AddElement(element, -1); 
}

function AddHandlers(adminManager)
{
    var handlerSection = adminManager.GetAdminSection("system.webServer/handlers", "MACHINE/WEBROOT/APPHOST");

    for (var i = 0; i < wcfHandlers.length; i++) 
    {
        if (wcfHandlers[i]._mode == "integrated") 
        {
            AddIntegratedHandler(handlerSection, wcfHandlers[i]._name, wcfHandlers[i]._path);
        }
        else if (wcfHandlers[i]._mode == "classic") 
        {
            AddISAPIHandler(handlerSection, wcfHandlers[i]._name, wcfHandlers[i]._path, wcfHandlers[i]._bitness );
        }
        else 
        {
            throw new Error("Unrecognized mode [" + wcfHandlers[i]._mode + "]");
        }
    }
}

function AddIntegratedHandler(section, name, path)
{
    var element = section.Collection.CreateNewElement("add"); 
    element.Properties.Item("name").Value = name; 
    element.Properties.Item("path").Value = path; 
    element.Properties.Item("verb").Value = "*"; 
    element.Properties.Item("type").Value = "System.ServiceModel.Activation.HttpHandler, System.ServiceModel, Version=3.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089";
    element.Properties.Item("preCondition").Value = "integratedMode,runtimeVersionv2.0"; 
    section.Collection.AddElement(element, 0); 
}

function AddISAPIHandler(section, name, path, bitness)
{
    var element = section.Collection.CreateNewElement("add");
    var scriptProcessor = null;
    var preCondition = null;

    if (bitness == "x86") 
    {
        scriptProcessor = "%SystemRoot%\\Microsoft.NET\\Framework\\v2.0.50727\\aspnet_isapi.dll";
        preCondition = "classicMode,runtimeVersionv2.0,bitness32"; 
    }
    else if (bitness == "x64") 
    {
        scriptProcessor = "%SystemRoot%\\Microsoft.NET\\Framework64\\v2.0.50727\\aspnet_isapi.dll";
        preCondition = "classicMode,runtimeVersionv2.0,bitness64"; 
    }
    else 
    {
        throw new Error("Unrecognized bitness [" + bitness + "]");
    }

    element.Properties.Item("name").Value = name; 
    element.Properties.Item("path").Value = path; 
    element.Properties.Item("verb").Value = "*"; 
    element.Properties.Item("modules").Value = "IsapiModule";
    element.Properties.Item("scriptProcessor").Value = scriptProcessor;
    element.Properties.Item("preCondition").Value = preCondition;

    section.Collection.AddElement(element, 0); 
}

function GetAdminManager()
{
    try
    {
        var vermg = new ActiveXObject("Microsoft.IIS.VersionManager");
        var exp = vermg.GetVersionObject("10.0", 1);
        return adminManager = exp.CreateObjectFromProgId("Microsoft.ApplicationHost.WritableAdminManager");
    }
    catch(e)
    {
        throw new Error("Unable to create WritableAdminManager.\r\n Please ensure that IIS Express is installed properly.\r\n\r\n  " + e.description);
    }
}

function FindElement(collection, elementTagName, valuesToMatch) 
{ 
    for (var i = 0; i < collection.Count; i++) 
    { 
        var element = collection.Item(i); 
         
        if (element.Name == elementTagName) 
        { 
            var matches = true; 
            for (var iVal = 0; iVal < valuesToMatch.length; iVal += 2) 
            { 
                var property = element.GetPropertyByName(valuesToMatch[iVal]); 
                var value = property.Value; 
                if (value != null) 
                { 
                    value = value.toString(); 
                } 
                if (value != valuesToMatch[iVal + 1]) 
                { 
                    matches = false; 
                    break; 
                } 
            } 
            if (matches) 
            { 
                return i; 
            } 
        } 
    } 
     
    return -1; 
}

function ParseArguments()
{
    var mode = "";
    
    if (WScript.Arguments.Count() > 0)
    {
        if (WScript.Arguments.Item(0).toLowerCase() == "install")
        {
            mode="install";
        }
        else if (WScript.Arguments.Item(0).toLowerCase() == "uninstall")
        {
            mode="uninstall";
        }
    }
    
    return mode;
}

function PrintUsage()
{
    WScript.Echo("Usage:\r\n   WCF35Setup.js <cmd>\r\n\r\nDescription:\r\nAdministration utility that enables configuation of WCF 3.5 for IIS Express\r\n\r\nSupported Commands:\r\n install, uninstall\r\n\r\nSamples:\r\n WCF35Setup.js install\r\n WCF35Setup.js uninstall");
}

// SIG // Begin signature block
// SIG // MIIdggYJKoZIhvcNAQcCoIIdczCCHW8CAQExCzAJBgUr
// SIG // DgMCGgUAMGcGCisGAQQBgjcCAQSgWTBXMDIGCisGAQQB
// SIG // gjcCAR4wJAIBAQQQEODJBs441BGiowAQS9NQkAIBAAIB
// SIG // AAIBAAIBAAIBADAhMAkGBSsOAwIaBQAEFIi1ZLl7QdBL
// SIG // By8iPp3D6iTn0GZwoIIYYjCCBMMwggOroAMCAQICEzMA
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
// SIG // DQEJBDEWBBQmuzIbOEYfj9GY1hDm96mmGXFiyzBABgor
// SIG // BgEEAYI3AgEMMTIwMKAYgBYASQBJAFMAIABFAHgAcABy
// SIG // AGUAcwBzoRSAEmh0dHA6Ly93d3cuaWlzLm5ldDANBgkq
// SIG // hkiG9w0BAQEFAASCAQAEA9ukY3YIIiysU2xSWuhQqAo8
// SIG // lYy4T/+wI1eg47b81xoS2GnVWBssFrf3OCkFQEH8oXap
// SIG // gWS/07vyZ0atT0oaeC7g4bNVmzwZhlE8MDcAXN2Nie5L
// SIG // rd3g6QRuGDwZkd+qX+VQvVMQKruIrrOSVmyJLeQ4LzkS
// SIG // aJ9NNGqKpgVl+uDxyPSaFM9IplpH14ovg3OiQfYw9LVq
// SIG // 1wqoWJzw0sIc4L4qciY4NL0ks4BGh6c0p6V6an5t/lNR
// SIG // rCQcKkpDzv9x4mXI7ctdvaD2moV01npyuef/zIR3BAFn
// SIG // /2S9pBlTx2nzYy2JTexqWg51mz+knqq/fwHGWTcbkn6+
// SIG // kXmE0v3+oYICKDCCAiQGCSqGSIb3DQEJBjGCAhUwggIR
// SIG // AgEBMIGOMHcxCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpX
// SIG // YXNoaW5ndG9uMRAwDgYDVQQHEwdSZWRtb25kMR4wHAYD
// SIG // VQQKExVNaWNyb3NvZnQgQ29ycG9yYXRpb24xITAfBgNV
// SIG // BAMTGE1pY3Jvc29mdCBUaW1lLVN0YW1wIFBDQQITMwAA
// SIG // AJvgdDfLPU2NLgAAAAAAmzAJBgUrDgMCGgUAoF0wGAYJ
// SIG // KoZIhvcNAQkDMQsGCSqGSIb3DQEHATAcBgkqhkiG9w0B
// SIG // CQUxDxcNMTYwNjAzMjM0ODUwWjAjBgkqhkiG9w0BCQQx
// SIG // FgQUv23v8XwAcvcm3PRrUEav/+VWUX4wDQYJKoZIhvcN
// SIG // AQEFBQAEggEAiX/A9IVRSO7g27WTIEc8mD3E6s1d47b/
// SIG // REpk/9OPFEF3v428Acyc/bykQuaRTuGl9vgKrs02wk3R
// SIG // YI+NY9XnJ7TzDBUBBbBSEMvdxK99xNcmQQjX7V2ex9hU
// SIG // YVssj5VDYc4jsrH3Cq+0TjQ/PmjKa8o8wyyf/rW9LTFm
// SIG // 6QXqvZVQXv7Iwc2VjDKs0CUf6b24tUkX9deiR2h4t5Qq
// SIG // xhInSSWsDLq2TSTK95Lht/6IIOfIw+pSzfhC58Xq/pSv
// SIG // Ipkvi/fzf0O9AKFohOC9mirxSmoQ0EIITm8d3X9qKnmC
// SIG // vH2dXeJZ2PZdIeF6dqb8mgK8SHsJNwnkKhO+JPNO06Fm0g==
// SIG // End signature block
