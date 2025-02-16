import { configureStore } from '@reduxjs/toolkit';
import { reducer, reducerPath, middleware } from '../features/control-panel/controlPanelApi';

export const store = configureStore({
  reducer: {
    [reducerPath]: reducer,
  },
  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware().concat(middleware),
});
