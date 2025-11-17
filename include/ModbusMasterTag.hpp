//
// Created by kdluzynski on 06.10.2025.
//

#ifndef MODBUSMASTERTAG_HPP
#define MODBUSMASTERTAG_HPP
#include <algorithm>
#include <mutex>
#include <qlist.h>
#include <set>
#include <variant>
#include "ModbusMasterBase.hpp"
#include "ModbusRegisterBuffer.hpp"
#include "ModbusTag.hpp"
/** Use Cases
 * 1. Aktualizacja read wartości - może byc asynchronicznie. Gdy przyjda nowe wartosci to tylko aktualizacja w modelu
 * 2. Synchroniczna sekwencja - np. write, read, write np. przeprowadzenie backupu
 *      - to albo synchroniczne akcje - ale to bedzie powodowac problem z thread affinity w Qt
 *      - albo zwracamy promise/future - ale to nie bedzie dzialalo w przypadku embedded bo tam nie ma
 * 3. Wpisywanie czegoś do wielu rejestrów na raz
 * 4. Odczyt z wielu rejestrów na raz
 * 5. Wymiana jakiegoś bufora np. pliku. Czyli wpisujemy w wiele różnych typów na raz wartości.
 *
 * To może w takim razie taki updejt poprzedniego sterownika, że wpisujemy do bufora, ale na podstawie Tagu.
 * Czyli tworzymy taki writer/builder requestów, czyli wpisujemy wartości jakie chcemy wpisać, albo odczytać
 * On buduje na tej podstawie requesty i później możemy je sobie wykonywać.
 * Tylko jak później odczytać z nich wartości?
 * Najlepiej pewnie, jakby otrzymywać w odpowiedzi taki bufor, z którego można by było odczytać wartości pojedynczo
 * odnosząc się do tagów np. value = answer[CID_TAG1] i on by automatycznie konwertował to do naszych wartości
 *
 * A później można by to było łatwo przekształcić w polling?
 * Trzeba by było mieć jakiś cache. Te answery trzeba by było móc jakoś łączyć. Tylko jak?
 */
namespace eModbus {
    struct Tag;

    class MasterTag : public MasterBase {
    public:
        using MasterBase::MasterBase;
        using TagID = std::string;
        using TagMap = std::unordered_map<TagID, size_t>;
        using TagValue = std::string;
        using TagValueMap = std::map<TagID, TagValue>;

        struct Request {
            RegisterType registerType;
            uint16_t startAddress;
            uint16_t quantity;
        };

        void registerTags(const std::vector<Tag> &tagsToRegister) {
            // OK, wiec tagi w bazie danych musza byc koniecznie posortowane wedlug typu rejestru i numeru
            // Chyba ze zrobic osobny vektor/multimape ktory bedzie tak posortowany i bedzie sie odnosil do mapy z tagid
            // for (const auto &[id, tag] : tagsToRegister) {
            //     tagsDatabase.insert_or_assign(id, tag);
            // }
            clearTags();
            tagsDatabase = tagsToRegister;
            std::ranges::sort(tagsDatabase, [](const Tag &a, const Tag &b) {
                return (a.register_type < b.register_type) ||
                       (a.register_type == b.register_type && a.register_number < b.register_number);
            });

            for (size_t i = 0; i < tagsDatabase.size(); ++i) {
                // Use the element's key and its new index 'i'
                IDtoTagMap.emplace(TagID{tagsDatabase[i].key}, i);
            }
        }

        void clearTags() {
            IDtoTagMap.clear();
            tagsDatabase.clear();
        }

        void runPolling();

        using TagRef = std::reference_wrapper<const Tag>;

        RegisterBuffer read(const uint8_t slave_ID, std::span<const TagRef> tags) {
            std::vector<Request> requests = prepareReadRequests(tags);
            std::vector<RegisterBuffer> resp;
            for (auto [registerType, startAddress, quantity]: requests) {
                RegisterBuffer& buf = resp.emplace_back(startAddress,registerType,quantity);
                MasterBase::read(slave_ID,buf.view());
            }
        }

        RegisterBuffer read(const uint8_t slave_ID, std::initializer_list<TagRef> tags) {
            return read(slave_ID, std::span(tags));
        }

        std::vector<uint16_t> read(const uint8_t slave_ID, const std::span<TagID> tagIDs) {
            std::vector<Request> requests = prepareReadRequests(tagIDs);
            std::vector<uint16_t> responses;
            std::vector<eModbus::RegisterBufferView> resp;
            for (auto request: requests) {
                try {
                    auto response = MasterBase::read(slave_ID, request.registerType, request.startAddress,
                                                     request.quantity);
                    responses.insert(responses.end(), response.begin(), response.end());
                } catch (ModbusException &e) {
                }
                // Response response = sendRequest(request);
                // result.insert(parseReadResponse(response));
            }
            return responses;
            // return {};
        }

        // TagValueMap read(std::initializer_list<TagID>tagIDs) {
        //     //check cache for ready requests
        //
        //     //if none is found, then prepare requests
        //     std::vector<RegisterBufferView> requests = prepare_requests(tagsIDs);
        //     //send all the requests
        //     for (const auto& request : requests) {
        //         readRegisters(request.)
        //     }
        //
        //     //return map
        // }

        void write(TagValueMap values);

        static MasterTag TCP(IStreamDevice &serial_device) {
            MasterTag result(serial_device);
            result.isTCP = true;
            return result;
        }

        static MasterTag RTU(IStreamDevice &serial_device) {
            MasterTag result(serial_device);
            result.isTCP = false;
            return result;
        }

    private:
        //Potrzebuje, zeby tagsDatabase bylo jednoczesnie szybkie do znalezienia przez TagID, oraz przez registerNumber
        // i RegisterType. Najlepiej jeśli jeszcze byłoby posortowane według registerNumber i RegisterType

        std::vector<Tag> tagsDatabase;
        TagMap IDtoTagMap;
        std::set<TagID> excludedTags; //moze powinno to byc excludedregisters?
        std::array<std::set<uint16_t>, 4> excludedRegisters;
        bool excludedTagsChanged;

        void sortTags(std::vector<TagID> &tags) {
            std::ranges::sort(tags, [this](const TagID &a_id, const TagID &b_id) {
                bool a_found = IDtoTagMap.contains(a_id);
                bool b_found = IDtoTagMap.contains(b_id);

                // If neither tag is found, treat them as equivalent
                if (!a_found && !b_found) return false;
                if (!a_found) return false; // Treat 'a' as greater if not found
                if (!b_found) return true; // Treat 'b' as greater if not found

                const Tag &a = getTag(a_id);
                const Tag &b = getTag(b_id);
                // Both tags are found, perform normal comparison
                return (a.register_type < b.register_type) ||
                       (a.register_type == b.register_type && a.register_number < b.register_number);
            });
        }


        bool checkRegistersContinuity(const TagID &first_tag_id, const TagID &end_tag_id) noexcept {
            if (first_tag_id == end_tag_id)
                return true;
            if (!IDtoTagMap.contains(first_tag_id) || !IDtoTagMap.contains(end_tag_id))
                return false;

            // return true;
            /** tak naprawde najlepszym algorytmem bylo by wziecie pierwszego taga i przeiterowanie
             * po kolei po wszystkich tagach następnych i aktualizowanie jak daleko "siega" ten register length
             **/
            /** OK czyli potrzebuje tak czy inaczej, zeby wszystkie tagi w db były posortowane według numeru rejestru
             * i typu rejestru bo inaczej to nie ma sensu. Czyli poniższe powinno działać, jesli zamienic iteratory na
             * iteratory do glownej bazy a nie mapy ? Najlepiej byloby gdyby to byla jednak mapa
             */

            auto currentTagIterator = IDtoTagMap.find(first_tag_id);
            auto endTagIterator = IDtoTagMap.find(end_tag_id);

            auto &previousTag = currentTagIterator->second;
            auto &endTag = endTagIterator->second;
            // if (previousTag.register_type != endTag.register_type)
            //     return false;
            // if (previousTag.register_number < endTag.register_number)
            //     return false;
            //
            //
            // int currentRegisterEnd = previousTag.register_number + previousTag.register_length;
            // ++currentTagIterator;
            //
            // for (;currentTagIterator != endTagIterator; ++currentTagIterator) {
            //     //We reached end of database
            //     if (currentTagIterator == IDtoTagMap.end())
            //         return false;
            //
            //     const auto& currentTag = currentTagIterator->second;
            //     if (currentTag.register_type != previousTag.register_type)
            //         return false;
            //     if (previousTag.register_number > currentRegisterEnd)
            //         return false;
            //
            //     int newRegisterEnd = currentTag.register_number + currentTag.register_length;
            //     if (newRegisterEnd > currentRegisterEnd)
            //         currentRegisterEnd = newRegisterEnd;
            //
            // }
            //we reached end iterator successfuly
            return true;
        }

        std::vector<Request> prepareReadRequests(std::span<TagID> tags) {
            std::vector<Request> requests;

            if (tags.empty())return requests;

            // sortTags(tags);
            TagID previousTagID{};
            for (const TagID &currentTagID: tags) {
                if (!IDtoTagMap.contains(currentTagID))
                    continue;
                if (excludedTags.contains(currentTagID))
                    continue;

                const Tag &currentTag = getTag(currentTagID);

                if (requests.empty())
                    requests.push_back({
                        .registerType = currentTag.register_type,
                        .startAddress = currentTag.register_number,
                        .quantity = currentTag.register_length,
                    });

                Request &currentRequest = requests.back();

                bool isSameType = currentRequest.registerType == currentTag.register_type;
                int distance = currentTag.register_number - currentRequest.startAddress;
                //Check if the distance between the first register in the request and this one is less then the maximum for requests
                uint16_t currentRegisterEnd = std::max(
                    distance + currentTag.register_length, static_cast<int>(currentRequest.quantity));
                bool registerOffsetLessThanMax = currentRegisterEnd <= eModbus::MAX_MODBUS_REGISTERS;

                //Check if registers are continuous (if they are not, then the modbus client can reject request)
                bool registersSpaceContinuous = checkRegistersContinuity(previousTagID, currentTagID);

                //Add new position to existing request
                if (isSameType && registerOffsetLessThanMax && registersSpaceContinuous) {
                    // uint16_t last_tag_position = current_request.valueCount() - previous_tag.register_length;
                    // current_request.setValueCount(last_tag_position  + register_offset + current_tag.register_length);//increase size of the register to pull
                    currentRequest.quantity = currentRegisterEnd; //increase size of the register to pull
                } else {
                    requests.push_back({
                        .registerType = currentTag.register_type,
                        .startAddress = currentTag.register_number,
                        .quantity = currentTag.register_length
                    });
                }
            }

            return requests;
        }

        bool checkForExcludedRegisters(const RegisterType registerType, uint16_t firstRegisterNumber,uint16_t lastRegisterNumber) const {
            if (firstRegisterNumber > lastRegisterNumber)
                std::swap(firstRegisterNumber,lastRegisterNumber);
            bool excludedRegistersFound = false;
            for (const auto excludedRegisterNumber: excludedRegisters[static_cast<int>(registerType)]) {
                if (excludedRegisterNumber >= firstRegisterNumber && excludedRegisterNumber <= lastRegisterNumber) {
                    excludedRegistersFound = true;
                    break;
                }
            }
            return excludedRegistersFound;
        }

        std::vector<Request> prepareReadRequests(const std::span<const TagRef> tags) const {
            std::vector<Request> requests;

            if (tags.empty())return requests;

            std::vector sortedTags(tags.begin(), tags.end());
            std::ranges::sort(sortedTags, [](const TagRef first, const TagRef second) {
                const Tag& a = first;
                const Tag& b = second;
                return (a.register_type < b.register_type) ||
                       (a.register_type == b.register_type && a.register_number < b.register_number);
            });
            TagID previousTagID{};
            for (const Tag currentTag: tags) {
                if (requests.empty())
                    requests.push_back({
                        .registerType = currentTag.register_type,
                        .startAddress = currentTag.register_number,
                        .quantity = currentTag.register_length,
                    });

                Request &currentRequest = requests.back();

                bool isSameType = currentRequest.registerType == currentTag.register_type;
                int distance = currentTag.register_number - currentRequest.startAddress;
                //Check if the distance between the first register in the request and this one is less then the maximum for requests
                uint16_t currentRegisterEnd = std::max(
                    distance + currentTag.register_length, static_cast<int>(currentRequest.quantity));
                bool registerOffsetLessThanMax = currentRegisterEnd <= eModbus::MAX_MODBUS_REGISTERS;

                //Add new position to existing request
                if (isSameType && registerOffsetLessThanMax
                    && checkForExcludedRegisters(currentRequest.registerType,
                        currentRequest.startAddress, currentTag.register_number)) {
                    currentRequest.quantity = currentRegisterEnd; //increase size of the register to pull
                } else {
                    requests.push_back({
                        .registerType = currentTag.register_type,
                        .startAddress = currentTag.register_number,
                        .quantity = currentTag.register_length
                    });
                }
            }

            return requests;
        }

        Tag &getTag(const TagID &tagID) {
            return tagsDatabase[IDtoTagMap.at(tagID)];
        }
    };
}


#endif //MODBUSMASTERTAG_HPP
