import { createApi } from "@reduxjs/toolkit/query/react";
import ReconnectingWebSocket from "reconnectingwebsocket";

const initialState = {
  motors: {
    left: 0,
    right: 0,
    trimmer: false,
  },
};

async function onCacheEntryAdded(_, { updateCachedData, cacheEntryRemoved }) {
  let ws = null;

  try {
    ws = new ReconnectingWebSocket("ws://localhost:4041");

    ws.onmessage = (event) => {
      const motors = JSON.parse(event.data);

      updateCachedData((draft) => {
        draft.motors = motors.data;
      });
    };
  } catch {}

  await cacheEntryRemoved;

  ws.close();
}

export const api = createApi({
  baseQuery: () => ({ data: initialState }),
  queryFn: () => ({ data: initialState }),
  reducerPath: "wsApi",
  endpoints: (build) => ({
    getMessages: build.query({
      query: (channel) => `messages/${channel}`,
      onCacheEntryAdded,
    }),
  }),
});

export const { useGetMessagesQuery, reducer, reducerPath, middleware } = api;
