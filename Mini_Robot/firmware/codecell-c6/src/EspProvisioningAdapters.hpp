#pragma once
// ESP-IDF NVS adapter for the #159 configuration store. Together with
// EspRuntimeAdapters.hpp and main.cpp this is the only production code that names SDK
// APIs; the store logic in include/provisioning/ is host-tested with fakes.
//
// Namespace "mr_config" holds the two record slots and the transaction marker as blobs.
// NVS writes a key with power-loss recovery of the old or new value; the record CRC, the
// A/B generations and the marker additionally make a torn or corrupt slot detectable, keep
// the previous record after an interrupted write and refuse to arm on a damaged committed
// record (ConfigStore.hpp).
// Arduino initializes the default NVS partition before setup(); if it finds the
// partition unusable it reformats it, which this firmware reports as Unprovisioned.
#include "provisioning/ConfigStore.hpp"
#include <cstddef>
#include <cstdint>
#include <nvs.h>

namespace codecell::esp {

inline constexpr const char* kConfigNamespace = "mr_config";
inline constexpr const char* kSlotKeyA = "rec_a";
inline constexpr const char* kSlotKeyB = "rec_b";
inline constexpr const char* kMarkerKey = "txn";
// NimBLE-Arduino 2.3.6 persists bonds in this namespace (ble_store_nvs.c,
// NIMBLE_NVS_NAMESPACE). The image uses NimBLE (#163), so a factory reset
// erases it directly; this removes any bond an earlier image (for example the BLE
// fixture) left behind, matching NimBLEDevice::deleteAllBonds() for stored bonds.
inline constexpr const char* kNimbleBondNamespace = "nimble_bond";

class NvsRecordStorage final : public provisioning::IRecordStorage {
public:
    provisioning::StorageRead read(provisioning::Slot slot, std::span<std::uint8_t> buffer) noexcept override
    {
        return readKey(key(slot), buffer);
    }
    bool write(provisioning::Slot slot, std::span<const std::uint8_t> record) noexcept override
    {
        return writeKey(key(slot), record);
    }
    bool erase(provisioning::Slot slot) noexcept override { return eraseKey(key(slot)); }

    provisioning::StorageRead readMarker(std::span<std::uint8_t> buffer) noexcept override
    {
        return readKey(kMarkerKey, buffer);
    }
    bool writeMarker(std::span<const std::uint8_t> marker) noexcept override { return writeKey(kMarkerKey, marker); }
    bool eraseMarker() noexcept override { return eraseKey(kMarkerKey); }

    bool eraseControllerBonds() noexcept override
    {
        nvs_handle_t handle = 0;
        // Read-only probe first, so a missing namespace is not created just to erase it.
        const esp_err_t probed = nvs_open(kNimbleBondNamespace, NVS_READONLY, &handle);
        if (probed == ESP_ERR_NVS_NOT_FOUND) return true;
        if (probed != ESP_OK) return false;
        nvs_close(handle);
        if (nvs_open(kNimbleBondNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
        const bool ok = nvs_erase_all(handle) == ESP_OK && nvs_commit(handle) == ESP_OK;
        nvs_close(handle);
        return ok;
    }

private:
    static const char* key(provisioning::Slot slot) noexcept
    {
        return slot == provisioning::Slot::A ? kSlotKeyA : kSlotKeyB;
    }

    static provisioning::StorageRead readKey(const char* name, std::span<std::uint8_t> buffer) noexcept
    {
        nvs_handle_t handle = 0;
        const esp_err_t opened = nvs_open(kConfigNamespace, NVS_READONLY, &handle);
        if (opened == ESP_ERR_NVS_NOT_FOUND) return {provisioning::StorageStatus::NotFound, 0};
        if (opened != ESP_OK) return {provisioning::StorageStatus::Failed, 0};
        std::size_t length = buffer.size();
        const esp_err_t got = nvs_get_blob(handle, name, buffer.data(), &length);
        nvs_close(handle);
        if (got == ESP_OK) return {provisioning::StorageStatus::Ok, length};
        if (got == ESP_ERR_NVS_NOT_FOUND) return {provisioning::StorageStatus::NotFound, 0};
        if (got == ESP_ERR_NVS_INVALID_LENGTH) return {provisioning::StorageStatus::TooLarge, 0};
        return {provisioning::StorageStatus::Failed, 0};
    }

    static bool writeKey(const char* name, std::span<const std::uint8_t> value) noexcept
    {
        nvs_handle_t handle = 0;
        if (nvs_open(kConfigNamespace, NVS_READWRITE, &handle) != ESP_OK) return false;
        const bool ok = nvs_set_blob(handle, name, value.data(), value.size()) == ESP_OK && nvs_commit(handle) == ESP_OK;
        nvs_close(handle);
        return ok;
    }

    static bool eraseKey(const char* name) noexcept
    {
        nvs_handle_t handle = 0;
        const esp_err_t opened = nvs_open(kConfigNamespace, NVS_READWRITE, &handle);
        if (opened != ESP_OK) return false;
        const esp_err_t erased = nvs_erase_key(handle, name);
        const bool ok = (erased == ESP_OK || erased == ESP_ERR_NVS_NOT_FOUND) && nvs_commit(handle) == ESP_OK;
        nvs_close(handle);
        return ok;
    }
};
} // namespace codecell::esp
