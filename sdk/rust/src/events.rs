use crate::{Error, Result};
use reqwest::blocking::Response;
use std::io::{self, BufReader, Read};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Event {
    pub event: String,
    pub id: Option<String>,
    pub data: String,
}

pub struct Events {
    reader: BufReader<Response>,
    finished: bool,
    last_id: Option<String>,
    skip_lf: bool,
    first_line: bool,
}

impl Events {
    pub(crate) fn new(response: Response) -> Self {
        Self {
            reader: BufReader::new(response),
            finished: false,
            last_id: None,
            skip_lf: false,
            first_line: true,
        }
    }

    fn line(&mut self) -> io::Result<Option<String>> {
        let mut bytes = Vec::new();
        loop {
            let mut byte = [0];
            if self.reader.read(&mut byte)? == 0 {
                return Ok(None);
            }
            if self.skip_lf {
                self.skip_lf = false;
                if byte[0] == b'\n' {
                    continue;
                }
            }
            if byte[0] == b'\r' || byte[0] == b'\n' {
                self.skip_lf = byte[0] == b'\r';
                let mut line = String::from_utf8(bytes)
                    .map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?;
                if self.first_line {
                    self.first_line = false;
                    line = line.trim_start_matches('\u{feff}').to_owned();
                }
                return Ok(Some(line));
            }
            bytes.push(byte[0]);
        }
    }
}

impl Iterator for Events {
    type Item = Result<Event>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.finished {
            return None;
        }
        let mut event = String::new();
        let mut data = Vec::new();
        loop {
            let line = match self.line() {
                Ok(None) => {
                    self.finished = true;
                    return None;
                }
                Err(error) => {
                    self.finished = true;
                    return Some(Err(Error::Io(error)));
                }
                Ok(Some(line)) => line,
            };
            if line.is_empty() {
                if data.is_empty() {
                    event.clear();
                    continue;
                }
                return Some(Ok(Event {
                    event: if event.is_empty() {
                        "message".into()
                    } else {
                        event
                    },
                    id: self.last_id.clone(),
                    data: data.join("\n"),
                }));
            }
            if line.starts_with(':') {
                continue;
            }
            let (field, value) = line.split_once(':').unwrap_or((&line, ""));
            let value = value.strip_prefix(' ').unwrap_or(value);
            match field {
                "event" => event = value.to_owned(),
                "data" => data.push(value.to_owned()),
                "id" if !value.contains('\0') => self.last_id = Some(value.to_owned()),
                _ => {}
            }
        }
    }
}
