function doGet(e){
    // open the spreadsheet
    var ss = SpreadsheetApp.getActive();
  
    var year_month = Utilities.formatDate(new Date(), SpreadsheetApp.getActive().getSpreadsheetTimeZone(), "MM/yyyy")
    // use the 'id' parameter to differentiate between sheets
    var sheet = ss.getSheetByName(year_month);
    if (sheet == null){
      sheet = ss.insertSheet();
      sheet.setName(year_month);
      sheet.getRange(1, 1).setValue("Datum").setHorizontalAlignment("center");
      sheet.getRange(1, 2).setValue("Temperatura").setHorizontalAlignment("center");
      sheet.getRange(1, 3).setValue("Vlaga").setHorizontalAlignment("center");
      sheet.getRange(1, 4).setValue("Pritisk").setHorizontalAlignment("center");
      sheet.getRange(1, 5).setValue("Baterija").setHorizontalAlignment("center");
      sheet.getRange(1, 6).setValue("RSSI").setHorizontalAlignment("center");
    }
    
    // extract headers
    // getRange accepts row, col, number_of_rows and num_of_cols as argument
    // getLastColumn returns the position of the last column that has content
    var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
    
    // store the position of the last row
    var lastRow = sheet.getLastRow();
    
    var cell = sheet.getRange('a1');
    var col = 0;
  
    for (i in headers){
      
      // loop through the headers and if a parameter name matches the header name insert the value
      if (headers[i] == "Datum")
      {
        val = Utilities.formatDate(new Date(), SpreadsheetApp.getActive().getSpreadsheetTimeZone(), "dd.MM.yyyy HH:mm:ss")
      }
      else
      {
        val = e.parameter[headers[i]]; 
      }
      
      // append data to the last row
      cell.offset(lastRow, col).setValue(val);
      col++;
    }
    
    return ContentService.createTextOutput('success');
  }