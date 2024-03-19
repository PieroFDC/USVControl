#include <iostream>
#include <string>
#include <curl/curl.h>
#include <chrono>
#include <thread>
#include <utility>

size_t write_data(void *buffer, size_t size, size_t nmemb, void *userp) {
    auto *data = (std::string *)userp;
    data->append((char *)buffer, size * nmemb);
    return size * nmemb;
}

class ApiException : public std::exception {
public:
    explicit ApiException(std::string message) : message(std::move(message)) {}

    [[nodiscard]] const char *what() const noexcept override {
        return message.c_str();
    }

private:
    std::string message;
};

float get_declination(float lat, float lon) {
    std::string url = "https://www.ngdc.noaa.gov/geomag-web/calculators/calculateDeclination?lat1=" + std::to_string(lat) + "&lon1=" + std::to_string(lon) + "&key=zNEw7&resultFormat=xml";

    CURL *curl = curl_easy_init();
    if(curl) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

        std::string response;

        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        int max_attempts = 3;
        int wait_time = 3;

        CURLcode res;

        for(int i = 0; i < max_attempts; i++) {
            res = curl_easy_perform(curl);

            if (res != CURLE_OK) {
                std::cerr << "Advertencia: Error al hacer la petición: " << curl_easy_strerror(res) << std::endl;

                if (i == max_attempts - 1) {
                    throw ApiException("Error: La llamada api falló después de " + std::to_string(max_attempts) + " intentos");

                } else {
                    std::this_thread::sleep_for(std::chrono::seconds(wait_time));
                }
            }
            else {
                break;
            }
        }

        curl_easy_cleanup(curl);

        std::string tag = "<declination units=\"Degree\">";
        size_t pos = response.find(tag);

        if (pos != std::string::npos) {
            std::string value = response.substr(pos + tag.length(), 8);

            return std::stof(value);

        } else {
            throw ApiException("Error: No se ha encontrado la etiqueta <declination> en la respuesta xml");
        }

    } else {
        throw ApiException("Error: No se ha podido inicializar la librería curl");
    }
}