#include "utils.h"

String decodeURIComponent(const String& encoded) {
  String decoded = encoded;
  
  decoded.replace("%20", " ");  // Espai
  decoded.replace("%21", "!");  // Exclamació
  decoded.replace("%22", "\""); // Cometes
  decoded.replace("%5C", "\\"); // Barra invertida literal
  decoded.replace("%23", "#");  // Hashtag
  decoded.replace("%24", "$");  // Dòlar
  decoded.replace("%25", "%");  // Percentatge
  decoded.replace("%26", "&");  // Amper
  decoded.replace("%27", "'");  // Cometes simples
  decoded.replace("%28", "(");  // Parèntesi esquerre
  decoded.replace("%29", ")");  // Parèntesi dreta
  decoded.replace("%2B", "+");  // Suma
  decoded.replace("%2C", ",");  // Coma
  decoded.replace("%2F", "/");  // Barra
  decoded.replace("%3A", ":");  // Dos punts
  decoded.replace("%3B", ";");  // Punt i coma
  decoded.replace("%3D", "=");  // Igual
  decoded.replace("%3F", "?");  // Interrogant
  decoded.replace("%40", "@");  // Arroba
  decoded.replace("%C3%A1", "á");  // À
  decoded.replace("%C3%A9", "é");  // É
  decoded.replace("%C3%AD", "í");  // Í
  decoded.replace("%C3%B3", "ó");  // Ó
  decoded.replace("%C3%BA", "ú");  // Ú
  decoded.replace("%C3%A0", "à");  // à
  decoded.replace("%C3%A8", "è");  // è
  decoded.replace("%C3%B2", "ò");   // ô
  decoded.replace("%C3%B4", "ô");  // ô
  decoded.replace("%C3%A7", "ç");  // ç
  decoded.replace("%C3%80", "À");  // À
  decoded.replace("%C3%89", "É");  // É
  decoded.replace("%C3%88", "È");  // È
  decoded.replace("%C3%92", "Ò");  // Ò
  decoded.replace("%C3%93", "Ó");  // Ó
  decoded.replace("%C3%9A", "Ú");  // Ú

  return decoded;
}

