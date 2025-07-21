use esp_idf_hal::delay::Delay;
use esp_idf_hal::gpio::{PinDriver, Gpio3, Gpio4, Output};
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::nvs::EspDefaultNvs;
use esp_idf_svc::wifi::{
    AccessPointConfiguration,
    ClientConfiguration,
    Configuration,
    EspWifi
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    nvs::{EspNvs, EspNvsPartition, NvsDefault, NvsPartitionId},
};
use esp_idf_sys::{esp_light_sleep_start, esp_sleep_enable_wifi_beacon_wakeup, esp_sleep_enable_wifi_wakeup, EspError};
use log::info;
use serde::{Deserialize, Serialize};

const NVS_NAMESPACE: &str = "app_config";
const NVS_KEY_SSID: &str = "ssid";
const NVS_KEY_PASS: &str = "password";
const NVS_KEY_MQTT_URL: &str = "mqtt_url";
const NVS_KEY_MQTT_TOPIC: &str = "mqtt_topic";

#[derive(Debug, Serialize, Deserialize)]
pub struct AppConfig {
    pub ssid: String,
    pub password: String,
    pub mqtt_url: String,
    pub mqtt_topic: String,
}

pub fn get_nvs_string<T: NvsPartitionId>(
    nvs: &mut EspNvs<T>,
    key: &str,
) -> anyhow::Result<Option<String>, EspError> {
    let required_len = nvs.str_len(key)?.unwrap_or(0);

    if required_len == 0 {
        return Ok(None);
    }

    let mut buf = vec![0; required_len as usize + 1];

    let s = nvs.get_str(key, &mut buf)?;

    Ok(s.map(String::from))
}

fn load_config(nvs: &mut EspNvs<NvsDefault>) -> anyhow::Result<Option<AppConfig>> {
    let ssid = get_nvs_string(nvs, NVS_KEY_SSID)?;
    let password = get_nvs_string(nvs, NVS_KEY_PASS)?;
    let mqtt_url = get_nvs_string(nvs, NVS_KEY_MQTT_URL)?;
    let mqtt_topic = get_nvs_string(nvs, NVS_KEY_MQTT_TOPIC)?;

    if let (Some(ssid), Some(password), Some(mqtt_url), Some(mqtt_topic)) =
        (ssid, password, mqtt_url, mqtt_topic)
    {
        Ok(Some(AppConfig {
            ssid,
            password,
            mqtt_url,
            mqtt_topic,
        }))
    } else {
        Ok(None)
    }
}

fn save_config(nvs: &mut EspNvs<NvsDefault>, config: &AppConfig) -> anyhow::Result<()> {
    nvs.set_str(NVS_KEY_SSID, &config.ssid)?;
    nvs.set_str(NVS_KEY_PASS, &config.password)?;
    nvs.set_str(NVS_KEY_MQTT_URL, &config.mqtt_url)?;
    nvs.set_str(NVS_KEY_MQTT_TOPIC, &config.mqtt_topic)?;
    Ok(())
}

fn main() -> anyhow::Result<()> {
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take()?;
    let delay = Delay::new_default();

    let modem = peripherals.modem;

    let mut red_led = PinDriver::output(peripherals.pins.gpio3)?;
    let mut green_led = PinDriver::output(peripherals.pins.gpio4)?;

    red_led.set_low()?;
    green_led.set_low()?;

    let nvs_default_partition = esp_idf_svc::nvs::EspNvsPartition::<NvsDefault>::take()?;
    let mut nvs = EspNvs::new(nvs_default_partition, NVS_NAMESPACE, true)?;

    let app_config_option = load_config(&mut nvs)?;

    match app_config_option {
        Some(config) => {
            info!("Configuration found in NVS. Starting operational mode");
            run_operation_mode(
                modem,
                sys_loop,
                delay,
                red_led,
                green_led
            )?;
        },
        None => {
            info!("No configuration found in NVS. Entering provisioning mode");
            run_provisioning_mode(
                modem,
                sys_loop,
                delay,
                red_led,
                green_led,
                nvs
            )?;
        }
    }

    Ok(())
}

fn run_operation_mode(
    modem: Modem,
    sys_loop: EspSystemEventLoop,
    delay: Delay,
    mut red_led: PinDriver<'_, Gpio3, Output>,
    mut green_led: PinDriver<'_, Gpio4, Output>
) -> anyhow::Result<()> {
    red_led.set_low()?;
    green_led.set_low()?;

    let mut wifi = EspWifi::new(
        modem,
        sys_loop.clone(),
        None
    )?;
    let wifi_config: Configuration = Configuration::Client(
        ClientConfiguration {
            ssid: "busyfi".try_into().unwrap(),
            bssid: None,
            auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
            password: "busyfi".try_into().unwrap(),
            channel: None,
            ..Default::default()
        }
    );
    wifi.set_configuration(&wifi_config)?;

    wifi.start()?;
    info!("Wifi started");

    wifi.connect()?;

    wifi.is_connected()?;
    info!("Wifi connected");

    // unsafe {
    //     esp_sleep_enable_wifi_beacon_wakeup();
    // }
    // info!("Wifi modem sleep enabled for Light-sleep.");

    // Skip MQTT for now

    loop {
        let status = true;
        //let status = *current_status.lock().unwrap();
        if status {
            red_led.set_high()?;
            green_led.set_low()?;
        } else {
            green_led.set_high()?;
            red_led.set_low()?;
        }

        delay.delay_ms(50);
        unsafe {
            esp_light_sleep_start();
        }
    }
}

fn run_provisioning_mode(
    modem: Modem,
    sys_loop: EspSystemEventLoop,
    delay: Delay,
    mut red_led: PinDriver<'_, Gpio3, Output>,
    mut green_led: PinDriver<'_, Gpio4, Output>,
    mut nvs: EspNvs<NvsDefault>
) -> anyhow::Result<()> {
    red_led.set_high()?;
    green_led.set_high()?;
    info!("LEDs are both lit");

    let mut wifi = EspWifi::new(
        modem,
        sys_loop.clone(),
        None
    )?;
    let wifi_config: Configuration = Configuration::AccessPoint(
        AccessPointConfiguration {
            ssid: "busyfi".try_into().unwrap(),
            password: "busyfi".try_into().unwrap(),
            auth_method: esp_idf_svc::wifi::AuthMethod::WPA2Personal,
            channel: 1,
            max_connections: 4,
            ssid_hidden: false,
            ..Default::default()
        }
    );
    wifi.set_configuration(&wifi_config)?;

    wifi.start()?;
    info!("Wifi started");

    loop {
        delay.delay_ms(1000); // Keep alive, let HTTP server handle requests
    }
}