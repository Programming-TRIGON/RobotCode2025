// TypeScript version of the original JavaScript code, including type annotations and features for enhanced clarity and robustness.

// @ts-ignore
import { serialize, deserialize } from "./msgpack.js";

// Lookup for type string indices
const typestrIdxLookup = {
  boolean: 0,
  double: 1,
  int: 2,
  float: 3,
  string: 4,
  json: 4,
  raw: 5,
  rpc: 5,
  msgpack: 5,
  protobuf: 5,
  "boolean[]": 16,
  "double[]": 17,
  "int[]": 18,
  "float[]": 19,
  "string[]": 20,
} as const;
// Type for the topic type, which is a string that can be one of the keys of the typestrIdxLookup object
export type TopicType = keyof typeof typestrIdxLookup

interface SubscriptionOptionsObj {
  periodic: number;
  all: boolean;
  topicsonly: boolean;
  prefix: boolean;
}

class NT4_SubscriptionOptions {
  periodic: number = 0.1;
  all: boolean = false;
  topicsOnly: boolean = false;
  prefix: boolean = false;

  toObj(): SubscriptionOptionsObj {
    return {
      periodic: this.periodic,
      all: this.all,
      topicsonly: this.topicsOnly,
      prefix: this.prefix,
    };
  }
}

interface SubscriptionObj {
  topics: string[];
  subuid: number;
  options: SubscriptionOptionsObj;
}

class NT4_Subscription {
  uid: number = -1;
  topics: Set<string> = new Set();
  options: NT4_SubscriptionOptions = new NT4_SubscriptionOptions();

  toSubscribeObj(): SubscriptionObj {
    return {
      topics: Array.from(this.topics),
      subuid: this.uid,
      options: this.options.toObj(),
    };
  }

  toUnsubscribeObj() {
    return {
      subuid: this.uid,
    };
  }
}

interface TopicProperties {
  [key: string]: any;
}

export class NT4_Topic {
  uid: number = -1; // "id" if server topic, "pubuid" if published
  name: string = "";
  type: TopicType = "double";
  properties: TopicProperties = {};

  toPublishObj() {
    return {
      name: this.name,
      type: this.type,
      pubuid: this.uid,
      properties: this.properties,
    };
  }

  toUnpublishObj() {
    return {
      pubuid: this.uid,
    };
  }

  getTypeIdx() {
    if (this.type in typestrIdxLookup) {
      return typestrIdxLookup[this.type];
    } else {
      return 5; // Default to binary
    }
  }
}

type OnTopicAnnounce = (topic: NT4_Topic) => void;
type OnTopicUnannounce = (topic: NT4_Topic) => void;
type OnNewTopicData = (topic: NT4_Topic, timestamp_us: number, value: any) => void;
type OnConnect = () => void;
type OnDisconnect = () => void;
type OnLatencyUpdate = (latency_us: number) => void;

export class NT4_Client {
  appName: string;
  onTopicAnnounce: OnTopicAnnounce;
  onTopicUnannounce: OnTopicUnannounce;
  onNewTopicData: OnNewTopicData;
  onConnect: OnConnect;
  onDisconnect: OnDisconnect;
  onLatencyUpdate: OnLatencyUpdate;
  serverBaseAddr: string;
  ws: WebSocket | null = null;
  serverAddr: string = "";
  serverConnectionActive: boolean = false;
  serverConnectionRequested: boolean = false;
  serverTimeOffset_us: number | null = null;
  networkLatency_us: number = 0;
  rxLengthCounter: number = 0;
  subscriptions: Map<number, NT4_Subscription> = new Map();
  publishedTopics: Map<string, NT4_Topic> = new Map();
  serverTopics: Map<string, NT4_Topic> = new Map();

  /**
   * Creates a new NT4 client without connecting.
   * @param serverAddr Network address of NT4 server
   * @param appName Identifier for this client (does not need to be unique).
   * @param onTopicAnnounce Gets called when server announces enough topics to form a new signal
   * @param onTopicUnannounce Gets called when server unannounces any part of a signal
   * @param onNewTopicData Gets called when any new data is available
   * @param onConnect Gets called once client completes initial handshake with server
   * @param onDisconnect Gets called once client detects server has disconnected
   */
  constructor(
    serverAddr: string,
    appName: string,
    onTopicAnnounce: OnTopicAnnounce,
    onTopicUnannounce: OnTopicUnannounce,
    onNewTopicData: OnNewTopicData,
    onConnect: OnConnect,
    onDisconnect: OnDisconnect,
    OnLatencyUpdate: OnLatencyUpdate
  ) {
    this.serverBaseAddr = serverAddr;
    this.appName = appName;
    this.onTopicAnnounce = onTopicAnnounce;
    this.onTopicUnannounce = onTopicUnannounce;
    this.onNewTopicData = onNewTopicData;
    this.onConnect = onConnect;
    this.onDisconnect = onDisconnect;
    this.onLatencyUpdate = OnLatencyUpdate;

    setInterval(() => {
      // Update timestamp
      this.ws_sendTimestamp();
      // Log bitrate
      const bitrateKbPerSec = ((this.rxLengthCounter / 1000) * 8) / 5;
      this.rxLengthCounter = 0;
      console.log(`[NT4] Bitrate: ${Math.round(bitrateKbPerSec)} kb/s`);
    }, 5000);
  }

  //////////////////////////////////////////////////////////////
  // PUBLIC API
  /** Starts the connection. The client will reconnect automatically when disconnected. */
  connect() {
    if (!this.serverConnectionRequested) {
      this.serverConnectionRequested = true;
      this.ws_connect();
    }
  }

  /** Terminates the connection. */
  disconnect() {
    if (this.serverConnectionRequested) {
      this.serverConnectionRequested = false;
      if (this.serverConnectionActive && this.ws) {
        this.ws.close();
      }
    }
  }

  /**
   * Add a new subscription, reading value updates
   * @param topicPatterns A list of topics or prefixes to include in the subscription.
   * @param prefixMode If true, use patterns as prefixes. If false, only subscribe to topics that are an exact match.
   * @param sendAll If true, send all values. If false, only send the most recent value.
   * @param periodic How frequently to send updates (applies regardless of "sendAll" option)
   * @returns A subscription ID that can be used to unsubscribe.
   */
  subscribe(topicPatterns: string[], prefixMode: boolean, sendAll = false, periodic = 0.1): number {
    const newSub = new NT4_Subscription();
    newSub.uid = this.getNewUID();
    newSub.topics = new Set(topicPatterns);
    newSub.options.prefix = prefixMode;
    newSub.options.all = sendAll;
    newSub.options.periodic = periodic;
    this.subscriptions.set(newSub.uid, newSub);
    if (this.serverConnectionActive) {
      this.ws_subscribe(newSub);
    }
    return newSub.uid;
  }

  /**
   * Add a new subscription, reading only topic announcements (not values).
   * @param topicPatterns A list of topics or prefixes to include in the subscription.
   * @param prefixMode If true, use patterns as prefixes. If false, only subscribe to topics that are an exact match.
   * @returns A subscription ID that can be used to unsubscribe.
   */
  subscribeTopicsOnly(topicPatterns: string[], prefixMode: boolean): number {
    const newSub = new NT4_Subscription();
    newSub.uid = this.getNewUID();
    newSub.topics = new Set(topicPatterns);
    newSub.options.prefix = prefixMode;
    newSub.options.topicsOnly = true;
    this.subscriptions.set(newSub.uid, newSub);
    if (this.serverConnectionActive) {
      this.ws_subscribe(newSub);
    }
    return newSub.uid;
  }

  /** Given an existing subscription, unsubscribe from it. */
  unsubscribe(subscriptionId: number) {
    const subscription = this.subscriptions.get(subscriptionId);
    if (!subscription) {
      throw `Unknown subscription ID "${subscriptionId}"`;
    }
    this.subscriptions.delete(subscriptionId);
    if (this.serverConnectionActive) {
      this.ws_unsubscribe(subscription);
    }
  }

  /** Unsubscribe from all current subscriptions. */
  clearAllSubscriptions() {
    for (const subscriptionId of this.subscriptions.keys()) {
      this.unsubscribe(subscriptionId);
    }
  }

  /**
   * Set the properties of a particular topic.
   * @param topic The topic to update
   * @param properties The set of new properties
   */
  setProperties(topic: string, properties: TopicProperties) {
    const updateTopic = (toUpdate: NT4_Topic) => {
      for (const key of Object.keys(properties)) {
        const value = properties[key];
        if (value === null) {
          delete toUpdate.properties[key];
        } else {
          toUpdate.properties[key] = value;
        }
      }
    };

    const publishedTopic = this.publishedTopics.get(topic);
    if (publishedTopic) updateTopic(publishedTopic);

    const serverTopic = this.serverTopics.get(topic);
    if (serverTopic) updateTopic(serverTopic);

    // Send new properties to server
    if (this.serverConnectionActive) {
      this.ws_setproperties(topic, properties);
    }
  }

  /** Set whether a topic is persistent. */
  setPersistent(topic: string, isPersistent: boolean) {
    this.setProperties(topic, { persistent: isPersistent });
  }

  /** Set whether a topic is retained. */
  setRetained(topic: string, isRetained: boolean) {
    this.setProperties(topic, { retained: isRetained });
  }


  /** Publish a topic from this client with the provided name and type. Can be a new or existing. */
  publishTopic(topic: string, type: TopicType) {
    if (this.publishedTopics.has(topic)) {
      return;
    }
    const newTopic = new NT4_Topic();
    newTopic.name = topic;
    newTopic.uid = this.getNewUID();
    newTopic.type = type;
    this.publishedTopics.set(topic, newTopic);
    if (this.serverConnectionActive) {
      this.ws_publish(newTopic);
    }
  }

  /** Unpublish a previously-published topic from this client. */
  unpublishTopic(topic: string) {
    const topicObj = this.publishedTopics.get(topic);
    if (!topicObj) {
      throw `Topic "${topic}" not found`;
    }
    this.publishedTopics.delete(topic);
    if (this.serverConnectionActive) {
      this.ws_unpublish(topicObj);
    }
  }

  /** Send some new value to the server. The timestamp is the current time. */
  addSample(topic: string, value: any) {
    let timestamp = this.getServerTime_us();
    if (timestamp === null) timestamp = 0;
    this.addTimestampedSample(topic, timestamp, value);
  }

  /** Send some new timestamped value to the server. */
  addTimestampedSample(topic: string, timestamp: number, value: any) {
    const topicObj = this.publishedTopics.get(topic);
    if (!topicObj) {
      throw `Topic "${topic}" not found`;
    }
    const txData = serialize([topicObj.uid, timestamp, topicObj.getTypeIdx(), value]);
    this.ws_sendBinary(txData);
  }

  //////////////////////////////////////////////////////////////
  // Server/Client Time Sync Handling

  /** Returns the current client time in microseconds. */
  public getClientTime_us(): number {
    return new Date().getTime() * 1000;
  }

  /** Returns the current server time in microseconds (or null if unknown). */
  public getServerTime_us(clientTime?: number): number | null {
    if (this.serverTimeOffset_us === null) {
      return null;
    } else {
      return (clientTime === undefined ? this.getClientTime_us() : clientTime) + this.serverTimeOffset_us;
    }
  }

  /** Returns the current network latency in microseconds. */
  public getNetworkLatency_us(): number {
    return this.networkLatency_us;
  }

  private ws_sendTimestamp() {
    const timeToSend = this.getClientTime_us();
    const txData = serialize([-1, 0, typestrIdxLookup["int"], timeToSend]);
    this.ws_sendBinary(txData);
  }

  private ws_handleReceiveTimestamp(serverTimestamp: number, clientTimestamp: number) {
    const rxTime = this.getClientTime_us();

    // Recalculate server/client offset based on round trip time
    const rtt = rxTime - clientTimestamp;
    this.networkLatency_us = rtt / 2.0;
    const serverTimeAtRx = serverTimestamp + this.networkLatency_us;
    this.serverTimeOffset_us = serverTimeAtRx - rxTime;

    console.log(`[NT4] New server time: ${(this.getServerTime_us()! / 1000000.0).toString()}s with ${(this.networkLatency_us / 1000.0).toString()}ms latency`);
    this.onLatencyUpdate(this.networkLatency_us);
  }

  //////////////////////////////////////////////////////////////
  // Websocket Message Send Handlers

  private ws_subscribe(subscription: NT4_Subscription) {
    this.ws_sendJSON("subscribe", subscription.toSubscribeObj());
  }

  private ws_unsubscribe(subscription: NT4_Subscription) {
    this.ws_sendJSON("unsubscribe", subscription.toUnsubscribeObj());
  }

  private ws_publish(topic: NT4_Topic) {
    this.ws_sendJSON("publish", topic.toPublishObj());
  }

  private ws_unpublish(topic: NT4_Topic) {
    this.ws_sendJSON("unpublish", topic.toUnpublishObj());
  }

  private ws_setproperties(topic: string, newProperties: TopicProperties) {
    this.ws_sendJSON("setproperties", {
      name: topic,
      update: newProperties,
    });
  }

  private ws_sendJSON(method: string, params: object) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(
        JSON.stringify([
          {
            method: method,
            params: params,
          },
        ])
      );
    }
  }

  private ws_sendBinary(data: ArrayBuffer) {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(data);
    }
  }

  //////////////////////////////////////////////////////////////
  // Websocket connection Maintenance

  private ws_onOpen() {
    // Set the flag allowing general server communication
    this.serverConnectionActive = true;
    console.log(`[NT4] Connected with identity "${this.appName}"`);

    // Sync timestamps
    this.ws_sendTimestamp();

    // Publish any existing topics
    for (const topic of this.publishedTopics.values()) {
      this.ws_publish(topic);
    }

    // Subscribe to existing subscriptions
    for (const subscription of this.subscriptions.values()) {
      this.ws_subscribe(subscription);
    }

    // User connection-opened hook
    this.onConnect();
  }

  private ws_onClose(event: CloseEvent) {
    // Clear flags to stop server communication
    this.ws = null;
    this.serverConnectionActive = false;

    // User connection-closed hook
    this.onDisconnect();

    // Clear out any local cache of server state
    this.serverTopics.clear();
    if (event.reason !== "") {
      console.log("[NT4] Socket is closed: ", event.reason);
    }
    if (this.serverConnectionRequested) {
      setTimeout(() => this.ws_connect(), 500);
    }
  }

  private ws_onError() {
    if (this.ws) this.ws.close();
  }

  private ws_onMessage(event: MessageEvent) {
    if (typeof event.data === "string") {
      // JSON array
      this.rxLengthCounter += event.data.length;
      const msgData = JSON.parse(event.data);
      if (!Array.isArray(msgData)) {
        console.warn("[NT4] Ignoring text message, JSON parsing did not produce an array at the top level.");
        return;
      }
      msgData.forEach((msg) => {
        // Validate proper format of message
        if (typeof msg !== "object") {
          console.warn("[NT4] Ignoring text message, JSON parsing did not produce an object.");
          return;
        }
        if (!("method" in msg) || !("params" in msg)) {
          console.warn("[NT4] Ignoring text message, JSON parsing did not find all required fields.");
          return;
        }

        const method = msg["method"];
        const params = msg["params"];
        if (typeof method !== "string") {
          console.warn('[NT4] Ignoring text message, JSON parsing found "method", but it wasn\'t a string.');
          return;
        }
        if (typeof params !== "object") {
          console.warn('[NT4] Ignoring text message, JSON parsing found "params", but it wasn\'t an object.');
          return;
        }

        // Message validates reasonably, switch based on supported methods
        if (method === "announce") {
          const newTopic = new NT4_Topic();
          newTopic.uid = params.id;
          newTopic.name = params.name;
          newTopic.type = params.type;
          newTopic.properties = params.properties;
          this.serverTopics.set(newTopic.name, newTopic);
          this.onTopicAnnounce(newTopic);
        } else if (method === "unannounce") {
          const removedTopic = this.serverTopics.get(params.name);
          if (!removedTopic) {
            console.warn("[NT4] Ignoring unannounce, topic was not previously announced.");
            return;
          }
          this.serverTopics.delete(removedTopic.name);
          this.onTopicUnannounce(removedTopic);
        } else if (method === "properties") {
          const topic = this.serverTopics.get(params.name);
          if (!topic) {
            console.warn("[NT4] Ignoring set properties, topic was not previously announced.");
            return;
          }
          for (const key of Object.keys(params.update)) {
            const value = params.update[key];
            if (value === null) {
              delete topic.properties[key];
            } else {
              topic.properties[key] = value;
            }
          }
        } else {
          console.warn("[NT4] Ignoring text message - unknown method " + method);
          return;
        }
      });
    } else {
      // MSGPack
      this.rxLengthCounter += event.data.byteLength;
      deserialize(event.data, { multiple: true }).forEach((unpackedData: any) => {
        const topicID = unpackedData[0];
        const timestamp_us = unpackedData[1];
        // const typeIdx = unpackedData[2];
        const value = unpackedData[3];

        if (topicID >= 0) {
          let topic: NT4_Topic | null = null;
          for (const serverTopic of this.serverTopics.values()) {
            if (serverTopic.uid === topicID) {
              topic = serverTopic;
              break;
            }
          }
          if (!topic) {
            console.warn("[NT4] Ignoring binary data - unknown topic ID " + topicID.toString());
            return;
          }
          this.onNewTopicData(topic, timestamp_us, value);
        } else if (topicID === -1) {
          this.ws_handleReceiveTimestamp(timestamp_us, value);
        } else {
          console.warn("[NT4] Ignoring binary data - invalid topic ID " + topicID.toString());
        }
      });
    }
  }

  private ws_connect() {
    const port = 5810;
    const prefix = "ws://";
    this.serverAddr = `${prefix}${this.serverBaseAddr}:${port}/nt/${this.appName}`;
    this.ws = new WebSocket(this.serverAddr, "networktables.first.wpi.edu");
    this.ws.binaryType = "arraybuffer";
    this.ws.addEventListener("open", () => this.ws_onOpen());
    this.ws.addEventListener("message", (event) => this.ws_onMessage(event));
    this.ws.addEventListener("close", (event) => this.ws_onClose(event));
    this.ws.addEventListener("error", () => this.ws_onError());
  }

  //////////////////////////////////////////////////////////////
  // General utilities

  private getNewUID(): number {
    return Math.floor(Math.random() * 99999999);
  }
}
