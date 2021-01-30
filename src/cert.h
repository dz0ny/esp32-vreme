
#include <Arduino.h>

String root_ca =
    F("-----BEGIN CERTIFICATE-----\n"
      "MIIO+TCCDeGgAwIBAgIRAKMGuV8qwQRiBQAAAACFkXMwDQYJKoZIhvcNAQELBQAw\n"
      "QjELMAkGA1UEBhMCVVMxHjAcBgNVBAoTFUdvb2dsZSBUcnVzdCBTZXJ2aWNlczET\n"
      "MBEGA1UEAxMKR1RTIENBIDFPMTAeFw0yMTAxMDUxMjExMDhaFw0yMTAzMzAxMjEx\n"
      "MDdaMHExCzAJBgNVBAYTAlVTMRMwEQYDVQQIEwpDYWxpZm9ybmlhMRYwFAYDVQQH\n"
      "Ew1Nb3VudGFpbiBWaWV3MRMwEQYDVQQKEwpHb29nbGUgTExDMSAwHgYDVQQDDBcq\n"
      "Lmdvb2dsZXVzZXJjb250ZW50LmNvbTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IA\n"
      "BBqxPbYAicB/LMAFEBx/+pDJ7m89wBK1HN32zPNvdwcJ+opMp9qFDg+MPTfTmOpi\n"
      "9Ulb+g0IE39qbq9gFK6dTwOjggyEMIIMgDAOBgNVHQ8BAf8EBAMCB4AwEwYDVR0l\n"
      "BAwwCgYIKwYBBQUHAwEwDAYDVR0TAQH/BAIwADAdBgNVHQ4EFgQUr1UCn70z7lJN\n"
      "jJGK+9fJCHQzDXQwHwYDVR0jBBgwFoAUmNH4bhDrz5vsYJ8YkBug630J/SswaAYI\n"
      "KwYBBQUHAQEEXDBaMCsGCCsGAQUFBzABhh9odHRwOi8vb2NzcC5wa2kuZ29vZy9n\n"
      "dHMxbzFjb3JlMCsGCCsGAQUFBzAChh9odHRwOi8vcGtpLmdvb2cvZ3NyMi9HVFMx\n"
      "TzEuY3J0MIIKPwYDVR0RBIIKNjCCCjKCFyouZ29vZ2xldXNlcmNvbnRlbnQuY29t\n"
      "ghwqLmFwcHMuZ29vZ2xldXNlcmNvbnRlbnQuY29tgiQqLmFwcHNwb3QuY29tLnN0\n"
      "b3JhZ2UuZ29vZ2xlYXBpcy5jb22CPSouYXVkaW9ib29rLWFkZGl0aW9uYWwtbWF0\n"
      "ZXJpYWwtc3RhZ2luZy5nb29nbGV1c2VyY29udGVudC5jb22CNSouYXVkaW9ib29r\n"
      "LWFkZGl0aW9uYWwtbWF0ZXJpYWwuZ29vZ2xldXNlcmNvbnRlbnQuY29tgg4qLmJs\n"
      "b2dzcG90LmNvbYIRKi5icC5ibG9nc3BvdC5jb22CIiouY29tbW9uZGF0YXN0b3Jh\n"
      "Z2UuZ29vZ2xlYXBpcy5jb22CJCouY29tcG9zZXItZGV2Lmdvb2dsZXVzZXJjb250\n"
      "ZW50LmNvbYIjKi5jb21wb3Nlci1xYS5nb29nbGV1c2VyY29udGVudC5jb22CKCou\n"
      "Y29tcG9zZXItc3RhZ2luZy5nb29nbGV1c2VyY29udGVudC5jb22CICouY29tcG9z\n"
      "ZXIuZ29vZ2xldXNlcmNvbnRlbnQuY29tgikqLmNvbnRlbnQtc3RvcmFnZS1kb3du\n"
      "bG9hZC5nb29nbGVhcGlzLmNvbYIjKi5jb250ZW50LXN0b3JhZ2UtcDIuZ29vZ2xl\n"
      "YXBpcy5jb22CJyouY29udGVudC1zdG9yYWdlLXVwbG9hZC5nb29nbGVhcGlzLmNv\n"
      "bYIgKi5jb250ZW50LXN0b3JhZ2UuZ29vZ2xlYXBpcy5jb22CKiouZGF0YWZ1c2lv\n"
      "bi1hcGktZGV2Lmdvb2dsZXVzZXJjb250ZW50LmNvbYIuKi5kYXRhZnVzaW9uLWFw\n"
      "aS1zdGFnaW5nLmdvb2dsZXVzZXJjb250ZW50LmNvbYImKi5kYXRhZnVzaW9uLWFw\n"
      "aS5nb29nbGV1c2VyY29udGVudC5jb22CJiouZGF0YWZ1c2lvbi1kZXYuZ29vZ2xl\n"
      "dXNlcmNvbnRlbnQuY29tgioqLmRhdGFmdXNpb24tc3RhZ2luZy5nb29nbGV1c2Vy\n"
      "Y29udGVudC5jb22CIiouZGF0YWZ1c2lvbi5nb29nbGV1c2VyY29udGVudC5jb22C\n"
      "KCouZGF0YXByb2Mtc3RhZ2luZy5nb29nbGV1c2VyY29udGVudC5jb22CJSouZGF0\n"
      "YXByb2MtdGVzdC5nb29nbGV1c2VyY29udGVudC5jb22CICouZGF0YXByb2MuZ29v\n"
      "Z2xldXNlcmNvbnRlbnQuY29tgiQqLmRldi5hbXA0bWFpbC5nb29nbGV1c2VyY29u\n"
      "dGVudC5jb22CHCouZG91YmxlY2xpY2t1c2VyY29udGVudC5jb22CNSouZnVjaHNp\n"
      "YS11cGRhdGVzLWF1dG9wdXNoLXF1YWwuZ29vZ2xldXNlcmNvbnRlbnQuY29tgjAq\n"
      "LmZ1Y2hzaWEtdXBkYXRlcy1hdXRvcHVzaC5nb29nbGV1c2VyY29udGVudC5jb22C\n"
      "KyouZnVjaHNpYS11cGRhdGVzLWRldi5nb29nbGV1c2VyY29udGVudC5jb22CLyou\n"
      "ZnVjaHNpYS11cGRhdGVzLXN0YWdpbmcuZ29vZ2xldXNlcmNvbnRlbnQuY29tgicq\n"
      "LmZ1Y2hzaWEtdXBkYXRlcy5nb29nbGV1c2VyY29udGVudC5jb22CGyouZ2NjLmdv\n"
      "b2dsZXVzZXJjb250ZW50LmNvbYILKi5nZ3BodC5jb22CESouZ29vZ2xlZHJpdmUu\n"
      "Y29tghcqLmdvb2dsZXN5bmRpY2F0aW9uLmNvbYIUKi5nb29nbGV3ZWJsaWdodC5j\n"
      "b22CGyouZ3NjLmdvb2dsZXVzZXJjb250ZW50LmNvbYIhKi5ub3RlYm9va3MuZ29v\n"
      "Z2xldXNlcmNvbnRlbnQuY29tgiEqLnBpcGVsaW5lcy5nb29nbGV1c2VyY29udGVu\n"
      "dC5jb22CNCoucGxheWdyb3VuZC1pbnRlcm5hbC5hbXA0bWFpbC5nb29nbGV1c2Vy\n"
      "Y29udGVudC5jb22CKyoucGxheWdyb3VuZC5hbXA0bWFpbC5nb29nbGV1c2VyY29u\n"
      "dGVudC5jb22CJSoucHJvZC5hbXA0bWFpbC5nb29nbGV1c2VyY29udGVudC5jb22C\n"
      "ISouc2FmZWZyYW1lLmdvb2dsZXN5bmRpY2F0aW9uLmNvbYIfKi5zYWZlbnVwLmdv\n"
      "b2dsZXVzZXJjb250ZW50LmNvbYIfKi5zYW5kYm94Lmdvb2dsZXVzZXJjb250ZW50\n"
      "LmNvbYIhKi5zdG9yYWdlLWRvd25sb2FkLmdvb2dsZWFwaXMuY29tgh8qLnN0b3Jh\n"
      "Z2UtdXBsb2FkLmdvb2dsZWFwaXMuY29tghgqLnN0b3JhZ2UuZ29vZ2xlYXBpcy5j\n"
      "b22CECoudHJhbnNsYXRlLmdvb2eCJCoudHVmLWF1dG9wdXNoLmdvb2dsZXVzZXJj\n"
      "b250ZW50LmNvbYIfKi50dWYtZGV2Lmdvb2dsZXVzZXJjb250ZW50LmNvbYIjKi50\n"
      "dWYtc3RhZ2luZy5nb29nbGV1c2VyY29udGVudC5jb22CGyoudHVmLmdvb2dsZXVz\n"
      "ZXJjb250ZW50LmNvbYIMYmxvZ3Nwb3QuY29tgg9icC5ibG9nc3BvdC5jb22CIGNv\n"
      "bW1vbmRhdGFzdG9yYWdlLmdvb2dsZWFwaXMuY29tghpkb3VibGVjbGlja3VzZXJj\n"
      "b250ZW50LmNvbYIJZ2dwaHQuY29tgg9nb29nbGVkcml2ZS5jb22CFWdvb2dsZXVz\n"
      "ZXJjb250ZW50LmNvbYISZ29vZ2xld2VibGlnaHQuY29tgiVtYW5pZmVzdC5jLm1h\n"
      "aWwuZ29vZ2xldXNlcmNvbnRlbnQuY29tgiVtYW5pZmVzdC5saDMtZGEuZ29vZ2xl\n"
      "dXNlcmNvbnRlbnQuY29tgiVtYW5pZmVzdC5saDMtZGIuZ29vZ2xldXNlcmNvbnRl\n"
      "bnQuY29tgiVtYW5pZmVzdC5saDMtZGMuZ29vZ2xldXNlcmNvbnRlbnQuY29tgiVt\n"
      "YW5pZmVzdC5saDMtZGQuZ29vZ2xldXNlcmNvbnRlbnQuY29tgiVtYW5pZmVzdC5s\n"
      "aDMtZGUuZ29vZ2xldXNlcmNvbnRlbnQuY29tgiVtYW5pZmVzdC5saDMtZGYuZ29v\n"
      "Z2xldXNlcmNvbnRlbnQuY29tgiVtYW5pZmVzdC5saDMtZGcuZ29vZ2xldXNlcmNv\n"
      "bnRlbnQuY29tgiVtYW5pZmVzdC5saDMtZHouZ29vZ2xldXNlcmNvbnRlbnQuY29t\n"
      "giJtYW5pZmVzdC5saDMuZ29vZ2xldXNlcmNvbnRlbnQuY29tgh5tYW5pZmVzdC5s\n"
      "aDMucGhvdG9zLmdvb2dsZS5jb22CFnN0b3JhZ2UuZ29vZ2xlYXBpcy5jb22CG3N0\n"
      "b3JhZ2UubXRscy5nb29nbGVhcGlzLmNvbYIOdHJhbnNsYXRlLmdvb2cwIQYDVR0g\n"
      "BBowGDAIBgZngQwBAgIwDAYKKwYBBAHWeQIFAzAzBgNVHR8ELDAqMCigJqAkhiJo\n"
      "dHRwOi8vY3JsLnBraS5nb29nL0dUUzFPMWNvcmUuY3JsMIIBBAYKKwYBBAHWeQIE\n"
      "AgSB9QSB8gDwAHcAfT7y+I//iFVoJMLAyp5SiXkrxQ54CX8uapdomX4i8NcAAAF2\n"
      "0qw2CwAABAMASDBGAiEAp2Wp/jFxee2gy2j1p0WOb9po66Z2o9Zp3UrykI2HT4EC\n"
      "IQDYXCbb3bbPyMvT/v33VV94m12eY9EchgVQ2gBAXcJtjgB1AESUZS6w7s6vxEAH\n"
      "2Kj+KMDa5oK+2MsxtT/TM5a1toGoAAABdtKsNCEAAAQDAEYwRAIgB2rFasWBU5QM\n"
      "7MQbPrWD+N9aa9S5VdA1BYuBHAVTk2cCIHmQf45SttIPLWKVmJ1EJ5iRTCMEP4EN\n"
      "wAKpqCUHOpVTMA0GCSqGSIb3DQEBCwUAA4IBAQBS1gFwoxrDYgEEI9VLFrtYh6QU\n"
      "KeZj3qSnzUDQmY");

String urlencode(String str) {
  String encodedString = "";
  char c;
  char code0;
  char code1;

  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
   
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
  
    }
    yield();
  }
  return encodedString;
}

unsigned char h2int(char c) {
  if (c >= '0' && c <= '9') {
    return ((unsigned char)c - '0');
  }
  if (c >= 'a' && c <= 'f') {
    return ((unsigned char)c - 'a' + 10);
  }
  if (c >= 'A' && c <= 'F') {
    return ((unsigned char)c - 'A' + 10);
  }
  return (0);
}