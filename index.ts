import { connect } from 'ngrok';
import { config } from 'dotenv';
import { validate } from 'jsonschema';

import express from 'express';

import type { Server } from 'http';
import type { AddressInfo } from 'net';

config();

const dataSchema = {
	type: 'object',
	properties: {
		heartRateAmplitude: { type: 'string' },
		beatsPerMinute: { type: 'string' },
		bloodOxygen: { type: 'string' },
	},
	required: ['heartRateAmplitude', 'beatsPerMinute', 'bloodOxygen'],
};

const app = express();

app.get('/', (request, response) => {
	const validation = validate(request.query, dataSchema);

	if (validation.valid) {
		console.log(validation.instance);

		response.status(200).json({
			status: 'success',
			message: validation.instance,
		});
	} else {
		response.status(422).json({
			status: 'failure',
			message: validation.errors.map((error) => `${error.argument} ${error.message})`),
		});
	}
});

const listener: Server = app.listen(process.env.PORT, () => {
	const port = (listener.address() as AddressInfo).port;

	connect({
		addr: port,
		authtoken: process.env.AUTH_TOKEN,
	}).then(console.log);

	console.log('Listening on port ' + port);
});
