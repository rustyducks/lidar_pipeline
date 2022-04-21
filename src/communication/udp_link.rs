use std::{io, thread, time::Duration};
use std::sync::mpsc::{Receiver, Sender, TryRecvError};
use crate::communication::transport::Transport;
use crate::communication::link::{LinkMessage};
use std::net::{SocketAddr, UdpSocket};

pub fn run(socket: UdpSocket, to_send_channel: Receiver<LinkMessage>, received_channel: Sender<LinkMessage>) {
    let mut trans = Transport::new();

    let mut clients: Vec<SocketAddr> = vec![];

    loop {

        match to_send_channel.try_recv() {
            // Get adversarial pose from the channel and send them through UDP
            Ok(msg) => {
                #[cfg(feature = "proto_debug")]
                println!("send: {:?}", msg.to_proto().unwrap());
                let buf = Transport::encode(&msg);
                for c in &clients {
                    let _ = socket.send_to(&buf, c);
                }
            },
            _ => {}
        }

        let mut buffer: [u8; 50] = [0; 50];
        match socket.recv_from(&mut buffer){
            // Get own pose from UDP (actually any message) and push it to the channel
            Ok((nb, addr)) => {
                if !clients.contains(&addr) {
                    clients.push(addr);
                    println!("new client: {:?}", addr);
                }
                
                for msg in trans.put(&buffer[0..nb]) {
                    received_channel.send(msg).expect("Coordinator is down.");
                }
            },
            Err(ref e) if e.kind() == io::ErrorKind::WouldBlock => (),
            Err(e) => eprintln!("{:?}", e),
        };

        
        thread::sleep(Duration::from_micros(10));    
    }
}