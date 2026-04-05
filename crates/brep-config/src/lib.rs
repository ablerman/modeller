//! Persistent configuration store backed by SQLite.
//!
//! Keys are plain strings (e.g. `"window.geometry"`).
//! Values are stored as JSON, so any `serde::Serialize`/`Deserialize` type
//! can be used without schema changes.

use rusqlite::Connection;
use serde::{de::DeserializeOwned, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ConfigError {
    #[error("sqlite error: {0}")]
    Sqlite(#[from] rusqlite::Error),
    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),
    #[error("no config directory found")]
    NoConfigDir,
}

/// Opens (or creates) the config database and provides typed key-value access.
pub struct ConfigStore {
    conn: Connection,
}

impl ConfigStore {
    /// Open the database at `~/.config/brep-modeller/config.db`.
    /// Creates the file and schema if they don't exist yet.
    pub fn open() -> Result<Self, ConfigError> {
        let path = config_db_path().ok_or(ConfigError::NoConfigDir)?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).ok();
        }
        let conn = Connection::open(&path)?;
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS config (
                key   TEXT PRIMARY KEY NOT NULL,
                value TEXT NOT NULL
            );",
        )?;
        Ok(Self { conn })
    }

    /// Retrieve a value by key. Returns `None` if the key doesn't exist.
    pub fn get<T: DeserializeOwned>(&self, key: &str) -> Result<Option<T>, ConfigError> {
        let mut stmt = self
            .conn
            .prepare("SELECT value FROM config WHERE key = ?1")?;
        let mut rows = stmt.query([key])?;
        match rows.next()? {
            Some(row) => {
                let json: String = row.get(0)?;
                Ok(Some(serde_json::from_str(&json)?))
            }
            None => Ok(None),
        }
    }

    /// Store a value under a key, replacing any existing entry.
    pub fn set<T: Serialize>(&self, key: &str, value: &T) -> Result<(), ConfigError> {
        let json = serde_json::to_string(value)?;
        self.conn.execute(
            "INSERT OR REPLACE INTO config (key, value) VALUES (?1, ?2)",
            [key, &json],
        )?;
        Ok(())
    }
}

/// Returns the path to `~/.config/brep-modeller/config.db` (platform-aware).
fn config_db_path() -> Option<std::path::PathBuf> {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            std::env::var_os("HOME")
                .map(|h| std::path::PathBuf::from(h).join(".config"))
        })
        .or_else(|| std::env::var_os("APPDATA").map(std::path::PathBuf::from))?;
    Some(base.join("brep-modeller").join("config.db"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn in_memory() -> ConfigStore {
        let conn = Connection::open_in_memory().unwrap();
        conn.execute_batch(
            "CREATE TABLE IF NOT EXISTS config (
                key   TEXT PRIMARY KEY NOT NULL,
                value TEXT NOT NULL
            );",
        )
        .unwrap();
        ConfigStore { conn }
    }

    #[test]
    fn round_trip_string() {
        let store = in_memory();
        store.set("greeting", &"hello world").unwrap();
        let got: String = store.get("greeting").unwrap().unwrap();
        assert_eq!(got, "hello world");
    }

    #[test]
    fn round_trip_struct() {
        #[derive(Debug, PartialEq, serde::Serialize, serde::Deserialize)]
        struct Point { x: i32, y: i32 }

        let store = in_memory();
        store.set("pt", &Point { x: 10, y: 20 }).unwrap();
        let got: Point = store.get("pt").unwrap().unwrap();
        assert_eq!(got, Point { x: 10, y: 20 });
    }

    #[test]
    fn missing_key_returns_none() {
        let store = in_memory();
        let got: Option<String> = store.get("nope").unwrap();
        assert!(got.is_none());
    }

    #[test]
    fn overwrite_existing_key() {
        let store = in_memory();
        store.set("n", &1u32).unwrap();
        store.set("n", &2u32).unwrap();
        let got: u32 = store.get("n").unwrap().unwrap();
        assert_eq!(got, 2);
    }
}
