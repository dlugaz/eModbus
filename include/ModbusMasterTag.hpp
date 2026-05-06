//
// Created by kdluzynski on 06.10.2025.
//

#pragma once

#include <algorithm>
#include <mutex>
#include <set>
#include <variant>
#include "ModbusMasterBase.hpp"
#include "ModbusRegisterBuffer.hpp"
#include "ModbusTag.hpp"
#include "ModbusTagValue.hpp"
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

        using TagMap = std::unordered_map<TagID, size_t>;
        using TagValueMap = std::map<TagID, TagValue>;

        struct Request {
            RegisterType registerType;
            uint16_t startAddress;
            uint16_t quantity;
        	std::vector<TagID> tagIDs;
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

        // RegisterBuffer read(const uint8_t slaveID, std::span<const TagRef> tags) {
        //     std::vector<Request> requests = prepareReadRequests(tags);
        //     std::vector<RegisterBuffer> resp;
        //     for (auto [registerType, startAddress, quantity,tagIDs]: requests) {
        //         RegisterBuffer& buf = resp.emplace_back(startAddress,registerType,quantity);
        //         MasterBase::read(slaveID,buf.view());
        //     }
        //     return resp;
        // }
        //
        // TagValueMap read(const uint8_t slaveID, std::initializer_list<TagRef> tags) {
        //     return read(slaveID, std::span(tags));
        // }

    	TagValueMap read(const uint8_t slaveID, std::initializer_list<const TagID> tags) {
        	return read(slaveID, std::span(tags));
        }

    	TagValueMap read(const uint8_t slaveID, const std::span<const TagID> tagIDs) {
        	std::vector<Request> requests = prepareReadRequests(tagIDs);
        	TagValueMap result;
        	std::vector<eModbus::RegisterBufferView> resp;
        	for (auto& [registerType, startAddress, quantity,tags]: requests) {
        		try {
        			auto response = MasterBase::read(slaveID, registerType, startAddress,
													 quantity);
        			RegisterBufferView parser {startAddress,registerType,response};
        			//associate TagID with value
			        for (auto & tagID: tags) {
				        auto& tag = getTag(tagID);
			        	result.emplace(tagID,parser.get<TagValue>(tag));
			        }
        		} catch (ModbusException &e) {
        		}
        	}
        	return result;
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

        void write(const uint8_t slaveID, TagValueMap values,bool oneByOne = true) {
        	for (auto & [tagID,value] : values) {
        		const auto& info = getTag(tagID);
				MasterBase::write(slaveID,info.register_type,info.register_number,value.data());
        	}
        }

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
                const bool a_found = IDtoTagMap.contains(a_id);
                const bool b_found = IDtoTagMap.contains(b_id);

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
        	const auto itStart = IDtoTagMap.find(first_tag_id);
        	const auto itEnd = IDtoTagMap.find(end_tag_id);

        	if (itStart == IDtoTagMap.end() || itEnd == IDtoTagMap.end()) return false;
        	size_t startIndex = itStart->second;
        	size_t endIndex = itEnd->second;
        	if (startIndex > endIndex) std::swap(startIndex, endIndex);
        	const Tag& firstTag = tagsDatabase[startIndex];
        	const Tag& lastTag = tagsDatabase[endIndex];

        	// 3. Different register types can never be "continuous" in one request
        	if (firstTag.register_type != lastTag.register_type) return false;
        	uint16_t currentReach = firstTag.register_number + firstTag.register_length;

        	for (size_t i = startIndex + 1; i <= endIndex; ++i) {
        		const Tag& currentTag = tagsDatabase[i];

        		// Gap detected: current tag starts after the previous reach
        		if (currentTag.register_number > currentReach) {
        			return false;
        		}

        		// Update reach: some tags might overlap or be sub-ranges of others[cite: 1, 2]
        		uint16_t newReach = currentTag.register_number + currentTag.register_length;
        		if (newReach > currentReach) {
        			currentReach = newReach;
        		}
        	}

            return true;
        }


        std::vector<Request> prepareReadRequests(const std::span<const TagID> tags) {
            std::vector<Request> requests;

            if (tags.empty())return requests;

            const TagID previousTagID{};
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

                const bool isSameType = currentRequest.registerType == currentTag.register_type;
                const int distance = currentTag.register_number - currentRequest.startAddress;
                //Check if the distance between the first register in the request and this one is less then the maximum for requests
                const uint16_t currentRegisterEnd = std::max(
                    distance + currentTag.register_length, static_cast<int>(currentRequest.quantity));
                const bool registerOffsetLessThanMax = currentRegisterEnd <= eModbus::MAX_MODBUS_REGISTERS;

                //Check if registers are continuous (if they are not, then the modbus client can reject request)
                const bool registersSpaceContinuous = checkRegistersContinuity(previousTagID, currentTagID);

                //Add new position to existing request
                if (isSameType && registerOffsetLessThanMax && registersSpaceContinuous) {
                    // uint16_t last_tag_position = current_request.valueCount() - previous_tag.register_length;
                    // current_request.setValueCount(last_tag_position  + register_offset + current_tag.register_length);//increase size of the register to pull
                    currentRequest.quantity = currentRegisterEnd; //increase size of the register to pull
                	currentRequest.tagIDs.push_back(currentTagID);
                } else {
                    requests.push_back({
                        .registerType = currentTag.register_type,
                        .startAddress = currentTag.register_number,
                        .quantity = currentTag.register_length
                    });
                	requests.back().tagIDs.push_back(currentTagID);
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

        Tag &getTag(const TagID &tagID) {
            return tagsDatabase[IDtoTagMap.at(tagID)];
        }
    };
}

