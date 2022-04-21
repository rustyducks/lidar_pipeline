use crate::communication::protoduck_generated::messages as proto;
use protobuf::Message;


#[derive(Clone)]
pub struct LinkMessage {
    buffer: Vec<u8>
}

impl LinkMessage {
    pub fn from_bytes(buf: &[u8]) -> Self {
        let buffer = buf.iter().map(|c| *c).collect();
        LinkMessage { buffer}        
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buffer[..]
    }

    pub fn to_proto(&self) -> anyhow::Result<proto::Message>{
        let msg = Message::parse_from_bytes(&self.buffer[..])?;
        Ok(msg)
    }
}