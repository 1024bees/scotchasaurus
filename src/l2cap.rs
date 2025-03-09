use bt_hci::WriteHci;
use embassy_futures::join::join;
use embassy_time::{Duration, Timer};

use esp_storage::FlashStorage;
use trouble_host;
use trouble_host::prelude::*;

use embedded_storage::Storage;
/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 3; // Signal + att + CoC
                                     //
                                     //
const BLESSED_ADDR: usize = 0x1000;
pub async fn run<C, const L2CAP_MTU: usize>(controller: C)
where
    C: Controller,
{
    let mut flash = FlashStorage::new();
    // Hardcoded peripheral address
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    log::info!("Our address = {:?}", address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        mut runner,
        ..
    } = stack.build();

    let mut adv_data = [0; 31];
    AdStructure::encode_slice(
        &[AdStructure::Flags(
            LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED,
        )],
        &mut adv_data[..],
    )
    .unwrap();

    let mut scan_data = [0; 31];
    AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(b"TroubleTROUBLETORUBLE")],
        &mut scan_data[..],
    )
    .unwrap();

    let _ = join(runner.run(), async {
        loop {
            log::info!("Advertising, waiting for connection...");
            let advertiser = peripheral
                .advertise(
                    &Default::default(),
                    Advertisement::ConnectableScannableUndirected {
                        adv_data: &adv_data[..],
                        scan_data: &scan_data[..],
                    },
                )
                .await
                .unwrap();
            let conn = advertiser.accept().await.unwrap();

            log::info!("Connection established");

            let mut ch1 = L2capChannel::accept(&stack, &conn, &[0x2349], &Default::default())
                .await
                .unwrap();

            log::info!("L2CAP channel accepted");

            // A buffer for reading chunks. Must be <= L2CAP_MTU in length.
            let mut rx_buf = [0; L2CAP_MTU];
            let mut start = 0;

            // This could be replaced with a file I/O or ring buffer approach. For
            // demonstration, we’ll just push bytes onto an in-memory buffer.

            // Example logic: Keep reading until you decide to stop. For instance,
            // you can define a protocol where the central sends some file size or
            // "EOF" marker. For now, we'll break if an error or a special marker occurs.
            loop {
                match ch1.receive(&stack, &mut rx_buf).await {
                    Ok(mut len) if len > 0 => {
                        // We got `len` bytes in rx_buf[0..len].
                        // Append them to your buffer or handle them in streaming fashion.

                        // For demonstration, let's look for a special marker that signals "end":
                        if rx_buf.ends_with(b"<EOF>") {
                            log::info!("EOF marker received; finishing file reception.");
                            // Remove the marker from the final data if needed
                            let cutoff = len - b"<EOF>".len();
                            len -= "b<EOF>".len();
                        }
                        let flash_res = flash.write(start, rx_buf.as_slice());
                        if let Err(err) = flash_res {
                            log::warn!("We failed to write to flash with err {err:?}");
                        }
                    }
                    Ok(_) => {
                        // len == 0 means the other side might have closed. We can break.
                        log::warn!("Received empty data, possible channel close?");
                        break;
                    }
                    Err(e) => {
                        // An error means the channel might be closed or lost. We'll break.
                        log::error!("Error receiving L2CAP data: {:?}", e);
                        break;
                    }
                }
            }

            // Now `received_data` should contain the entire "file" that was sent
            // (minus any protocol markers you strip off). You can store it,
            // process it, or handle it as needed.

            // Possibly echo back a confirmation:
            let response = b"File received OK";
            ch1.send::<_, L2CAP_MTU>(&stack, response).await.unwrap();
            log::info!("Sent response to central.");

            // Wait a bit before advertising again or you might keep the connection
            // open if that’s your desired behavior.
            Timer::after(Duration::from_secs(5)).await;
        }
    })
    .await;
}
